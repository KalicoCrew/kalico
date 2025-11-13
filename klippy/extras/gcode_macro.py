# Add ability to define custom g-code macros
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations
import traceback, logging, ast, copy, json, threading
import jinja2, math
import typing
import functools
import importlib, importlib.metadata, importlib.util
import inspect
import pathlib
from klippy.printer import Printer
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
    def __init__(self, printer, env, name, script, script_type="gcode"):
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


BlockingResult = typing.TypeVar("BlockingResult")


class PythonMacroContext:
    'The magic "Printer" object for macros'

    status: dict[str, dict[str, typing.Any]]
    vars: dict[str, typing.Any]

    raw_params: str
    params: dict[str, str]

    def __init__(self, printer: Printer, name: str, context: dict):
        self._printer = printer
        self._gcode = printer.lookup_object("gcode")
        self._gcode_macro = printer.lookup_object(f"gcode_macro {name}")
        self._name = name

        self.status = GetStatusWrapperPython(printer)
        self.vars = TemplateVariableWrapperPython(self._gcode_macro)

        self.raw_params = context.get("raw_params", None)
        self.params = context.get("params", {})

    def emit(self, gcode: str):
        "Run GCode"
        self._gcode.run_gcode_from_command(gcode)

    def wait_while(self, condition: typing.Callable[[], bool]):
        "Wait while a condition is True"

        def inner(eventtime):
            return condition()

        self._printer.wait_while(inner)

    def wait_until(self, condition: typing.Callable[[], bool]):
        "Wait until a condition is True"

        def inner(eventtime):
            return not condition()

        self._printer.wait_until(condition)

    def wait_moves(self):
        "Wait until all moves are completed"
        if self._toolhead is None:
            self._toolhead = self._printer.lookup_object("toolhead")
        self._toolhead.wait_moves()

    def blocking(
        self, function: typing.Callable[[], BlockingResult]
    ) -> BlockingResult:
        "Run a blocking task in a thread, waiting for the result"
        completion = self._printer.get_reactor().completion()

        def run():
            try:
                ret = function()
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

    def sleep(self, timeout: float):
        "Wait a given number of seconds"
        reactor = self._printer.get_reactor()
        deadline = reactor.monotonic() + timeout

        def check(event):
            return deadline > reactor.monotonic()

        self._printer.wait_while(check)

    def set_gcode_variable(self, macro: str, variable: str, value: typing.Any):
        "Save a variable to a gcode_macro"
        macro = self._printer.lookup_object(f"gcode_macro {macro}")
        macro.variables = {**macro.variables, variable: value}

    def emergency_stop(self, msg: str = "action_emergency_stop"):
        "Immediately shutdown Kalico"
        self._printer.invoke_shutdown(f"Shutdown due to {msg}")

    def respond(self, prefix: str, msg: str):
        "Send a message to the console"
        self._gcode.respond_raw(f"{prefix} {msg}")

    def respond_info(self, msg: str):
        "Send a message to the console"
        self._gcode.respond_info(msg)

    def respond_raw(self, msg: str):
        self._gcode.respond_raw(msg)

    def raise_error(self, msg):
        "Raise a G-Code command error"
        raise self._printer.command_error(msg)

    def call_remote_method(self, method: str, **kwargs):
        "Call a Kalico webhooks method"
        webhooks = self._printer.lookup_object("webhooks")
        try:
            webhooks.call_remote_method(method, **kwargs)
        except self._printer.command_error:
            logging.exception("Remote call error")


class PythonMacroTemplate:
    def __init__(self, config, macro_func):
        self.printer = config.get_printer()

        self.name = macro_func.__macro_name__
        self.func = macro_func

    def create_template_context(self):
        return {}  # Shim

    def run_gcode_from_command(self, context=None):
        context = PythonMacroContext(self.printer, self.name, context)
        try:
            return self.func(context)
        except self.printer.command_error:
            raise
        except Exception as e:
            raise self.printer.command_error(f"Error in {self.name}: {e}")

    def __call__(self, context=None):
        return self.run_gcode_from_command(context)


macro_function = typing.Callable[
    typing.Concatenate[PythonMacroContext, ...], None
]


class PythonMacroLoader:
    def __init__(self, config: configfile.ConfigWrapper):
        self._printer = config.get_printer()
        self._config = config
        self._root_path = pathlib.Path(
            self._config.printer.get_start_args()["config_file"]
        ).parent

        self._filename = None
        self._loaded_macros = None

    def _build_macro_wrapper(self, name, func):
        sig = inspect.signature(func)
        if not sig.parameters:
            raise configfile.error(
                f"Error when loading macro {self._filename}:{name}."
                " Macro functions must accept a parameter for the Printer object"
            )

        param_gcmd, *parameters = sig.parameters.values()
        if param_gcmd.annotation not in (sig.empty, PythonMacroContext):
            raise configfile.error(
                f"Error when loading macro {self._filename}:{name}."
                " First parameter must be of type Printer"
            )

        macro_description = name.upper()
        for paramspec in parameters:
            if paramspec.annotation not in (
                paramspec.empty,
                str,
                int,
                bool,
                float,
            ):
                raise configfile.error(
                    f"Error when loading macro {self._filename}:{name}."
                    f" Parameter '{paramspec.name}: {paramspec.annotation}' may only be str, int, float, or bool"
                )

            param_description = paramspec.name.upper() + "="
            if paramspec.annotation is float:
                param_description += "1.23"
            elif paramspec.annotation is int:
                param_description += "123"
            elif paramspec.annotation is bool:
                if paramspec.default is True:
                    param_description += "0"
                elif paramspec.default is False:
                    param_description += "1"
                else:
                    param_description += "0|1"
            if paramspec.default is not paramspec.empty:
                param_description = "[" + param_description + "]"
            macro_description += " " + param_description

        docstring = macro_description
        if func.__doc__:
            docstring += "\n\n" + func.__doc__

        @functools.wraps(func)
        def _wrapper(context: PythonMacroContext):
            kwargs = {}
            print(context.params)
            for paramspec in parameters:
                param_name = paramspec.name.upper()

                if param_name not in context.params:
                    if paramspec.default is paramspec.empty:
                        raise context._printer.command_error(
                            f"{name} requires {param_name}\n", macro_description
                        )
                    else:
                        continue

                value = context.params[param_name]

                # Special case for boolean values
                if paramspec.annotation is bool:
                    value = bool(int(value))

                elif callable(paramspec.annotation):
                    value = paramspec.annotation(value)

                kwargs[paramspec.name] = value

            bound = sig.bind(context, **kwargs)
            func(*bound.args, **bound.kwargs)

        _wrapper.__macro_name__ = name
        _wrapper.__filename__ = self._filename
        _wrapper.__doc__ = macro_description

        return _wrapper

    def _macro_decorator(
        self,
        name: str,
        rename_existing: typing.Optional[str] = None,
    ) -> typing.Callable[[macro_function], macro_function]:
        def macro_decorator(func: macro_function) -> macro_function:
            wrapped_macro = self._build_macro_wrapper(name, func)

            self._register_macro(wrapped_macro, rename_existing)

            return func

        return macro_decorator

    def get_context(self):
        return {
            "Printer": PythonMacroContext,
            "config": self._config,
            "gcode_macro": self._macro_decorator,
        }

    def load(self, filename):
        self._filename = self._root_path / filename

        if not self._filename.exists():
            raise configfile.error(
                f"Error loading python macros: {self._filename} does not exist"
            )

        spec = importlib.util.spec_from_file_location(
            self._filename.name, self._filename
        )
        module = importlib.util.module_from_spec(spec)

        module.__dict__.update(self.get_context())
        spec.loader.exec_module(module)

    def _register_macro(self, macro_func, rename_existing=None):
        section = f"gcode_macro {macro_func.__macro_name__}"

        config = self._config.getsection(section)
        if not config.has_section(section):
            config.fileconfig.add_section(section)
            config.fileconfig.set(section, "description", macro_func.__doc__)
            if rename_existing:
                config.fileconfig.set(
                    section, "rename_existing", rename_existing
                )

        template = PythonMacroTemplate(config, macro_func)

        gcode_macro = self._printer.lookup_object(section, None)
        if gcode_macro:
            gcode_macro.template = template
        else:
            gcode_macro = GCodeMacro(config, template)
            self._printer.add_object(section, gcode_macro)


# Main gcode macro template tracking
class PrinterGCodeMacro:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
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

        self.load_macros_from_python(config)

    def load_macros_from_python(self, config):
        files = config.getlist("python", [], sep="\n")
        for file in files:
            if not file.strip():
                continue
            macro_loader = PythonMacroLoader(config)
            macro_loader.load(file)

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

        for name, obj in self.printer.lookup_objects("gcode_macro "):
            if not new_config.has_section(name):
                continue
            new_section = new_config.getsection(name)
            if name in [
                s.get_name()
                for s in new_config.get_prefix_sections("gcode_macro")
            ]:
                template = obj.template
                script_type, new_script = new_section.getscript("gcode")
                template.reload(script_type, new_script)

        # TODO: Verify this works like I expect
        self.load_macros_from_python(self.config)


def load_config(config):
    return PrinterGCodeMacro(config)


######################################################################
# GCode macro
######################################################################


class GCodeMacro:
    def __init__(self, config, template: Template = None):
        if len(config.get_name().split()) > 2:
            raise config.error(
                "Name of section '%s' contains illegal whitespace"
                % (config.get_name())
            )
        name = config.get_name().split()[1]
        self.alias = name.upper()
        self.printer = printer = config.get_printer()
        self.template = template
        if self.template is None:
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
