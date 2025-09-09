# Add ability to define custom g-code macros
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import traceback, logging, ast, copy, json, threading
import jinja2, math
import typing

from klippy import configfile


######################################################################
# Template handling
######################################################################


# Wrapper for access to printer object get_status() methods
class GetStatusWrapperJinja:
    def __init__(self, printer, eventtime=None):
        self.printer = printer
        self.eventtime = eventtime
        self.cache = {}

    def __getitem__(self, val):
        sval = str(val).strip()
        if sval in self.cache:
            return self.cache[sval]
        po = self.printer.lookup_object(sval, None)
        if po is None or not hasattr(po, "get_status"):
            raise KeyError(val)
        if self.eventtime is None:
            self.eventtime = self.printer.get_reactor().monotonic()
        self.cache[sval] = res = copy.deepcopy(po.get_status(self.eventtime))
        return res

    def __contains__(self, val):
        try:
            self.__getitem__(val)
        except KeyError as e:
            return False
        return True

    def __iter__(self):
        for name, obj in self.printer.lookup_objects():
            if self.__contains__(name):
                yield name


class GetStatusWrapperPython:
    def __init__(self, printer):
        self.printer = printer

    def __getitem__(self, val):
        sval = str(val).strip()
        po = self.printer.lookup_object(sval, None)
        if po is None or not hasattr(po, "get_status"):
            raise KeyError(val)
        eventtime = self.printer.get_reactor().monotonic()
        return po.get_status(eventtime)

    def __getattr__(self, val):
        return self.__getitem__(val)

    def __contains__(self, val):
        try:
            self.__getitem__(val)
        except KeyError as e:
            return False
        return True

    def __iter__(self):
        for name, obj in self.printer.lookup_objects():
            if self.__contains__(name):
                yield name

    def get(self, key: str, default: configfile.sentinel):
        try:
            return self[key]
        except KeyError:
            if default is not configfile.sentinel:
                return default
            raise


# Wrapper around a Jinja2 template
class TemplateWrapperJinja:
    def __init__(self, printer, env, name, script):
        self.printer = printer
        self.name = name
        self.gcode = self.printer.lookup_object("gcode")
        gcode_macro = self.printer.lookup_object("gcode_macro")
        self.create_template_context = gcode_macro.create_template_context
        try:
            self.template = env.from_string(script)
        except jinja2.exceptions.TemplateSyntaxError as e:
            lines = script.splitlines()
            msg = "Error loading template '%s'\nline %s: %s # %s" % (
                name,
                e.lineno,
                lines[e.lineno - 1],
                e.message,
            )
            logging.exception(msg)
            raise self.gcode.error(msg)
        except Exception as e:
            msg = "Error loading template '%s' (line: %s): %s" % (
                name,
                e.lineno,
                traceback.format_exception_only(type(e), e)[-1],
            )
            logging.exception(msg)
            raise printer.config_error(msg)

    def render(self, context=None):
        if context is None:
            context = self.create_template_context()
        try:
            return str(self.template.render(context))
        except Exception as e:
            msg = "Error evaluating '%s': %s" % (
                self.name,
                traceback.format_exception_only(type(e), e)[-1],
            )
            logging.exception(msg)
            raise self.gcode.error(msg)

    def run_gcode_from_command(self, context=None):
        self.gcode.run_script_from_command(self.render(context))


class TemplateWrapperPython:
    def __init__(self, printer, env, name, script):
        self.printer = printer
        self.name = name
        self.toolhead = None
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode_macro = self.printer.lookup_object("gcode_macro")
        self.create_template_context = self.gcode_macro.create_template_context
        self.checked_own_macro = False
        self.vars = None

        try:
            self.func = compile(script, name, "exec")
        except SyntaxError as e:
            msg = "Error compiling expression '%s': %s at line %d column %d" % (
                self.name,
                traceback.format_exception_only(type(e), e)[-1],
                e.lineno,
                e.offset,
            )
            logging.exception(msg)
            raise self.gcode.error(msg)

    def run_gcode_from_command(self, context=None):
        helpers = {
            "printer": GetStatusWrapperPython(self.printer),
            "emit": self._action_emit,
            "wait_while": self._action_wait_while,
            "wait_until": self._action_wait_until,
            "wait_moves": self._action_wait_moves,
            "blocking": self._action_blocking,
            "sleep": self._action_sleep,
            "set_gcode_variable": self._action_set_gcode_variable,
            "emergency_stop": self.gcode_macro._action_emergency_stop,
            "respond_info": self.gcode_macro._action_respond_info,
            "raise_error": self.gcode_macro._action_raise_error,
            "call_remote_method": self.gcode_macro._action_call_remote_method,
            "action_emergency_stop": self.gcode_macro._action_emergency_stop,
            "action_respond_info": self.gcode_macro._action_respond_info,
            "action_raise_error": self.gcode_macro._action_raise_error,
            "action_call_remote_method": self.gcode_macro._action_call_remote_method,
            "math": math,
        }

        if not self.checked_own_macro:
            self.checked_own_macro = True
            own_macro = self.printer.lookup_object(
                self.name.split(":")[0], None
            )
            if own_macro is not None and isinstance(own_macro, GCodeMacro):
                self.vars = TemplateVariableWrapperPython(own_macro)
        if self.vars is not None:
            helpers["own_vars"] = self.vars

        if context is None:
            context = {}
        exec_globals = dict(context | helpers)
        try:
            exec(self.func, exec_globals, {})
        except Exception as e:
            msg = "Error evaluating '%s': %s" % (
                self.name,
                traceback.format_exception_only(type(e), e)[-1],
            )
            logging.exception(msg)
            raise self.gcode.error(msg)

    def _action_emit(self, gcode):
        self.gcode.run_script_from_command(gcode)

    def _action_wait_while(self, check):
        def inner(eventtime):
            return check()

        self.printer.wait_while(check)

    def _action_wait_until(self, check):
        def inner(eventtime):
            return not check()

        self.printer.wait_while(inner)

    def _action_wait_moves(self):
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object("toolhead")
        self.toolhead.wait_moves()

    def _action_blocking(self, func):
        completion = self.printer.get_reactor().completion()

        def run():
            try:
                ret = func()
                completion.complete((False, ret))
            except Exception as e:
                completion.complete((True, e))

        t = threading.Thread(target=run, daemon=True)
        t.start()
        [is_exception, ret] = completion.wait()
        if is_exception:
            raise ret
        else:
            return ret

    def _action_sleep(self, timeout):
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + timeout

        def check(event):
            return deadline > reactor.monotonic()

        self.printer.wait_while(check)

    def _action_set_gcode_variable(self, macro, variable, value):
        macro = self.printer.lookup_object(f"gcode_macro {macro}")
        v = dict(macro.variables)
        v[variable] = value
        macro.variables = v


class TemplateVariableWrapperPython:
    def __init__(self, macro):
        self.__dict__["__macro"] = macro

    def __setattr__(self, name, value):
        v = dict(self.__dict__["__macro"].variables)
        v[name] = value
        self.__dict__["__macro"].variables = v

    def __getattr__(self, name):
        if name not in self.__dict__["__macro"].variables:
            raise KeyError(name)
        return self.__dict__["__macro"].variables[name]

    def __contains__(self, val):
        try:
            self.__getattr__(val)
        except KeyError as e:
            return False
        return True

    def __iter__(self):
        for name, obj in self.__dict__["__macro"].variables:
            yield name


class Template:
    def __init__(self, printer, env, name, script, script_type="gcode") -> None:
        self.name = name
        self.printer = printer
        self.env = env
        self.reload(script_type, script)

    def __call__(self, context=None):
        return self.function(context)

    def __getattr__(self, name):
        return getattr(self.function, name)

    def reload(
        self,
        script_type: typing.Literal["python", "gcode"],
        script: str,
    ):
        if script_type == "python":
            self.function = TemplateWrapperPython(
                self.printer, self.env, self.name, script
            )
        else:
            self.function = TemplateWrapperJinja(
                self.printer, self.env, self.name, script
            )


# Main gcode macro template tracking
class PrinterGCodeMacro:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.env = jinja2.Environment(
            "{%",
            "%}",
            "{",
            "}",
            extensions=[
                "jinja2.ext.do",
                "jinja2.ext.loopcontrols",
            ],
        )

        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "RELOAD_GCODE_MACROS", self.cmd_RELOAD_GCODE_MACROS
        )

    def load_template(self, config, option, default=None):
        name = "%s:%s" % (config.get_name(), option)
        if default is None:
            script_type, script = config.getscript(option)
        else:
            script_type, script = config.getscript(option, default)
        return Template(
            self.printer, self.env, name, script, script_type=script_type
        )

    def _action_emergency_stop(self, msg="action_emergency_stop"):
        self.printer.invoke_shutdown("Shutdown due to %s" % (msg,))
        return ""

    def _action_respond_info(self, msg):
        self.printer.lookup_object("gcode").respond_info(msg)
        return ""

    def _action_log(self, msg):
        logging.info(msg)
        return ""

    def _action_raise_error(self, msg):
        raise self.printer.command_error(msg)

    def _action_call_remote_method(self, method, **kwargs):
        webhooks = self.printer.lookup_object("webhooks")
        try:
            webhooks.call_remote_method(method, **kwargs)
        except self.printer.command_error:
            logging.exception("Remote Call Error")
        return ""

    def create_template_context(self, eventtime=None):
        return {
            "printer": GetStatusWrapperJinja(self.printer, eventtime),
            "action_emergency_stop": self._action_emergency_stop,
            "action_respond_info": self._action_respond_info,
            "action_log": self._action_log,
            "action_raise_error": self._action_raise_error,
            "action_call_remote_method": self._action_call_remote_method,
            "math": math,
        }

    def cmd_RELOAD_GCODE_MACROS(self, gcmd):
        pconfig = configfile.PrinterConfig(self.printer)
        new_config = pconfig.read_main_config()
        for name, obj in self.printer.lookup_objects("gcode_macro"):
            try:
                new_section = new_config.getsection(name)
            except:
                continue

            if name in [
                s.get_name()
                for s in new_config.get_prefix_sections("gcode_macro")
            ]:
                template = obj.template
                script_type, new_script = new_section.getscript("gcode")
                template.reload(script_type, new_script)


def load_config(config):
    return PrinterGCodeMacro(config)


######################################################################
# GCode macro
######################################################################


class GCodeMacro:
    def __init__(self, config):
        if len(config.get_name().split()) > 2:
            raise config.error(
                "Name of section '%s' contains illegal whitespace"
                % (config.get_name())
            )
        name = config.get_name().split()[1]
        self.alias = name.upper()
        self.printer = printer = config.get_printer()
        gcode_macro = printer.load_object(config, "gcode_macro")
        self.template = gcode_macro.load_template(config, "gcode")
        self.gcode = printer.lookup_object("gcode")
        self.rename_existing = config.get("rename_existing", None)
        self.cmd_desc = config.get("description", "G-Code macro")
        if self.rename_existing is not None:
            if self.gcode.is_traditional_gcode(
                self.alias
            ) != self.gcode.is_traditional_gcode(self.rename_existing):
                raise config.error(
                    "G-Code macro rename of different types ('%s' vs '%s')"
                    % (self.alias, self.rename_existing)
                )
            printer.register_event_handler(
                "klippy:connect", self.handle_connect
            )
        else:
            self.gcode.register_command(
                self.alias, self.cmd, desc=self.cmd_desc
            )
        self.gcode.register_mux_command(
            "SET_GCODE_VARIABLE",
            "MACRO",
            name,
            self.cmd_SET_GCODE_VARIABLE,
            desc=self.cmd_SET_GCODE_VARIABLE_help,
        )
        self.in_script = False
        self.variables = {}
        prefix = "variable_"
        for option in config.get_prefix_options(prefix):
            try:
                literal = ast.literal_eval(config.get(option))
                json.dumps(literal, separators=(",", ":"))
                self.variables[option[len(prefix) :]] = literal
            except (SyntaxError, TypeError, ValueError) as e:
                raise config.error(
                    "Option '%s' in section '%s' is not a valid literal: %s"
                    % (option, config.get_name(), e)
                )

    def handle_connect(self):
        prev_cmd = self.gcode.register_command(self.alias, None)
        if prev_cmd is None:
            raise self.printer.config_error(
                "Existing command '%s' not found in gcode_macro rename"
                % (self.alias,)
            )
        pdesc = "Renamed builtin of '%s'" % (self.alias,)
        self.gcode.register_command(self.rename_existing, prev_cmd, desc=pdesc)
        self.gcode.register_command(self.alias, self.cmd, desc=self.cmd_desc)

    def get_status(self, eventtime):
        return self.variables

    cmd_SET_GCODE_VARIABLE_help = "Set the value of a G-Code macro variable"

    def cmd_SET_GCODE_VARIABLE(self, gcmd):
        variable = gcmd.get("VARIABLE")
        value = gcmd.get("VALUE")
        if variable not in self.variables:
            raise gcmd.error("Unknown gcode_macro variable '%s'" % (variable,))
        try:
            literal = ast.literal_eval(value)
            json.dumps(literal, separators=(",", ":"))
        except (SyntaxError, TypeError, ValueError) as e:
            raise gcmd.error(
                "Unable to parse '%s' as a literal: %s" % (value, e)
            )
        v = dict(self.variables)
        v[variable] = literal
        self.variables = v

    def cmd(self, gcmd):
        if self.in_script:
            raise gcmd.error("Macro %s called recursively" % (self.alias,))
        kwparams = dict(self.variables)
        kwparams.update(self.template.create_template_context())
        kwparams["params"] = gcmd.get_command_parameters()
        kwparams["rawparams"] = gcmd.get_raw_command_parameters()
        self.in_script = True
        try:
            self.template.run_gcode_from_command(kwparams)
        finally:
            self.in_script = False


def load_config_prefix(config):
    return GCodeMacro(config)
