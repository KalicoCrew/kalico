#!/usr/bin/env python2
# Script to handle build time requests embedded in C code.
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, subprocess, optparse, logging, shlex, socket, time, traceback
import json, zlib

sys.path.append("./klippy")
import msgproto  # noqa: E402

FILEHEADER = """
/* DO NOT EDIT! This is an autogenerated file. See scripts/buildcommands.py. */

#include "board/irq.h"
#include "board/pgm.h"
#include "command.h"
#include "compiler.h"
#include "initial_pins.h"
"""


def error(msg):
    sys.stderr.write(msg + "\n")
    sys.exit(-1)


Handlers = []


######################################################################
# C call list generation
######################################################################


# Create dynamic C functions that call a list of other C functions
class HandleCallList:
    def __init__(self):
        self.call_lists = {"ctr_run_initfuncs": []}
        self.ctr_dispatch = {"_DECL_CALLLIST": self.decl_calllist}

    def decl_calllist(self, req):
        funcname, callname = req.split()[1:]
        self.call_lists.setdefault(funcname, []).append(callname)

    def update_data_dictionary(self, data):
        pass

    def generate_code(self, options):
        code = []
        for funcname, funcs in self.call_lists.items():
            func_code = [
                "    extern void %s(void);\n    %s();" % (f, f) for f in funcs
            ]
            if funcname == "ctr_run_taskfuncs":
                add_poll = "    irq_poll();\n"
                func_code = [add_poll + fc for fc in func_code]
                func_code.append(add_poll)
            fmt = """
void
%s(void)
{
    %s
}
"""
            code.append(fmt % (funcname, "\n".join(func_code).strip()))
        return "".join(code)


Handlers.append(HandleCallList())


######################################################################
# Enumeration and static string generation
######################################################################

STATIC_STRING_MIN = 2


# Generate a dynamic string to integer mapping
class HandleEnumerations:
    def __init__(self):
        self.static_strings = []
        self.enumerations = {}
        self.ctr_dispatch = {
            "_DECL_STATIC_STR": self.decl_static_str,
            "DECL_ENUMERATION": self.decl_enumeration,
            "DECL_ENUMERATION_RANGE": self.decl_enumeration_range,
        }

    def add_enumeration(self, enum, name, value):
        enums = self.enumerations.setdefault(enum, {})
        if name in enums and enums[name] != value:
            error(
                "Conflicting definition for enumeration '%s %s'" % (enum, name)
            )
        enums[name] = value

    def decl_enumeration(self, req):
        enum, name, value = req.split()[1:]
        self.add_enumeration(enum, name, int(value, 0))

    def decl_enumeration_range(self, req):
        enum, name, value, count = req.split()[1:]
        self.add_enumeration(enum, name, (int(value, 0), int(count, 0)))

    def decl_static_str(self, req):
        msg = req.split(None, 1)[1]
        if msg not in self.static_strings:
            self.static_strings.append(msg)

    def update_data_dictionary(self, data):
        for i, s in enumerate(self.static_strings):
            self.add_enumeration("static_string_id", s, i + STATIC_STRING_MIN)
        data["enumerations"] = self.enumerations

    def generate_code(self, options):
        code = []
        for i, s in enumerate(self.static_strings):
            code.append(
                '    if (__builtin_strcmp(str, "%s") == 0)\n'
                "        return %d;\n" % (s, i + STATIC_STRING_MIN)
            )
        fmt = """
uint8_t __always_inline
ctr_lookup_static_string(const char *str)
{
    %s
    return 0xff;
}
"""
        return fmt % ("".join(code).strip(),)


HandlerEnumerations = HandleEnumerations()
Handlers.append(HandlerEnumerations)


######################################################################
# Constants
######################################################################


# Allow adding build time constants to the data dictionary
class HandleConstants:
    def __init__(self):
        self.constants = {}
        self.ctr_dispatch = {
            "DECL_CONSTANT": self.decl_constant,
            "DECL_CONSTANT_STR": self.decl_constant_str,
        }

    def set_value(self, name, value):
        if name in self.constants and self.constants[name] != value:
            error("Conflicting definition for constant '%s'" % name)
        self.constants[name] = value

    def decl_constant(self, req):
        name, value = req.split()[1:]
        self.set_value(name, int(value, 0))

    def decl_constant_str(self, req):
        name, value = req.split(None, 2)[1:]
        value = value.strip()
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
        self.set_value(name, value)

    def update_data_dictionary(self, data):
        data["config"] = self.constants

    def generate_code(self, options):
        return ""


HandlerConstants = HandleConstants()
Handlers.append(HandlerConstants)


######################################################################
# Initial pins
######################################################################


class HandleInitialPins:
    def __init__(self):
        self.initial_pins = []
        self.ctr_dispatch = {"DECL_INITIAL_PINS": self.decl_initial_pins}

    def decl_initial_pins(self, req):
        pins = req.split(None, 1)[1].strip()
        if pins.startswith('"') and pins.endswith('"'):
            pins = pins[1:-1]
        if pins:
            self.initial_pins = [p.strip() for p in pins.split(",")]
            HandlerConstants.decl_constant_str(
                "_DECL_CONSTANT_STR INITIAL_PINS " + ",".join(self.initial_pins)
            )

    def update_data_dictionary(self, data):
        pass

    def map_pins(self):
        if not self.initial_pins:
            return []
        mp = msgproto.MessageParser()
        mp.fill_enumerations(HandlerEnumerations.enumerations)
        pinmap = mp.get_enumerations().get("pin", {})
        out = []
        for p in self.initial_pins:
            flag = "IP_OUT_HIGH"
            if p.startswith("!"):
                flag = "0"
                p = p[1:].strip()
            if p not in pinmap:
                error("Unknown initial pin '%s'" % (p,))
            out.append("\n    {%d, %s}, // %s" % (pinmap[p], flag, p))
        return out

    def generate_code(self, options):
        out = self.map_pins()
        fmt = """
const struct initial_pin_s initial_pins[] PROGMEM = {%s
};
const int initial_pins_size PROGMEM = ARRAY_SIZE(initial_pins);
"""
        return fmt % ("".join(out),)


Handlers.append(HandleInitialPins())


######################################################################
# ARM IRQ vector table generation
######################################################################


# Create ARM IRQ vector table from interrupt handler declarations
class Handle_arm_irq:
    def __init__(self):
        self.irqs = {}
        self.ctr_dispatch = {"DECL_ARMCM_IRQ": self.decl_armcm_irq}

    def decl_armcm_irq(self, req):
        func, num = req.split()[1:]
        num = int(num, 0)
        if num in self.irqs and self.irqs[num] != func:
            error(
                "Conflicting IRQ definition %d (old %s new %s)"
                % (num, self.irqs[num], func)
            )
        self.irqs[num] = func

    def update_data_dictionary(self, data):
        pass

    def generate_code(self, options):
        armcm_offset = 16
        if 1 - armcm_offset not in self.irqs:
            # The ResetHandler was not defined - don't build VectorTable
            return ""
        max_irq = max(self.irqs.keys())
        table = ["    DefaultHandler,\n"] * (max_irq + armcm_offset + 1)
        defs = []
        for num, func in self.irqs.items():
            if num < 1 - armcm_offset:
                error("Invalid IRQ %d (%s)" % (num, func))
            defs.append("extern void %s(void);\n" % (func,))
            table[num + armcm_offset] = "    %s,\n" % (func,)
        table[0] = "    &_stack_end,\n"
        fmt = """
extern void DefaultHandler(void);
extern uint32_t _stack_end;
%s
const void *VectorTable[] __visible __section(".vector_table") = {
%s};
"""
        return fmt % ("".join(defs), "".join(table))


Handlers.append(Handle_arm_irq())


######################################################################
# Wire protocol commands and responses
######################################################################


# Dynamic command and response registration
class HandleCommandGeneration:
    def __init__(self):
        self.commands = {}
        self.encoders = []
        self.msg_to_encid = dict(msgproto.DefaultMessages)
        self.encid_to_msgid = {}
        self.messages_by_name = {m.split()[0]: m for m in self.msg_to_encid}
        self.all_param_types = {}
        self.ctr_dispatch = {
            "DECL_COMMAND_FLAGS": self.decl_command,
            "_DECL_ENCODER": self.decl_encoder,
            "_DECL_OUTPUT": self.decl_output,
        }

    def decl_command(self, req):
        funcname, flags, msgname = req.split()[1:4]
        if msgname in self.commands:
            error("Multiple definitions for command '%s'" % msgname)
        self.commands[msgname] = (funcname, flags, msgname)
        msg = req.split(None, 3)[3]
        m = self.messages_by_name.get(msgname)
        if m is not None and m != msg:
            error("Conflicting definition for command '%s'" % msgname)
        self.messages_by_name[msgname] = msg

    def decl_encoder(self, req):
        msg = req.split(None, 1)[1]
        msgname = msg.split()[0]
        m = self.messages_by_name.get(msgname)
        if m is not None and m != msg:
            error("Conflicting definition for message '%s'" % msgname)
        self.messages_by_name[msgname] = msg
        self.encoders.append((msgname, msg))

    def decl_output(self, req):
        msg = req.split(None, 1)[1]
        self.encoders.append((None, msg))

    def convert_encoded_msgid(self, encoded_msgid):
        if encoded_msgid >= 0x80:
            data = [(encoded_msgid >> 7) | 0x80, encoded_msgid & 0x7F]
        else:
            data = [encoded_msgid]
        return msgproto.PT_int32().parse(data, 0)[0]

    def create_message_ids(self):
        # Create unique ids for each message type
        encoded_msgid = max(self.msg_to_encid.values())
        mlist = list(self.commands.keys()) + [m for n, m in self.encoders]
        for msgname in mlist:
            msg = self.messages_by_name.get(msgname, msgname)
            if msg not in self.msg_to_encid:
                encoded_msgid += 1
                self.msg_to_encid[msg] = encoded_msgid
        if encoded_msgid >= 1 << 14:
            # The mcu currently assumes all message ids encode to 1 or 2 bytes
            error("Too many message ids")
        self.encid_to_msgid = {
            encoded_msgid: self.convert_encoded_msgid(encoded_msgid)
            for encoded_msgid in self.msg_to_encid.values()
        }

    def update_data_dictionary(self, data):
        # Convert ids to standard form (use both positive and negative numbers)
        msg_to_msgid = {
            msg: self.encid_to_msgid[encoded_msgid]
            for msg, encoded_msgid in self.msg_to_encid.items()
        }
        command_ids = [
            msg_to_msgid[msg]
            for msgname, msg in self.messages_by_name.items()
            if msgname in self.commands
        ]
        response_ids = [
            msg_to_msgid[msg]
            for msgname, msg in self.messages_by_name.items()
            if msgname not in self.commands
        ]
        data["commands"] = {
            msg: msgid
            for msg, msgid in msg_to_msgid.items()
            if msgid in command_ids
        }
        data["responses"] = {
            msg: msgid
            for msg, msgid in msg_to_msgid.items()
            if msgid in response_ids
        }
        output = {
            msg: msgid
            for msg, msgid in msg_to_msgid.items()
            if msgid not in command_ids and msgid not in response_ids
        }
        if output:
            data["output"] = output

    def build_parser(self, encoded_msgid, msgformat, msgtype):
        if msgtype == "output":
            param_types = msgproto.lookup_output_params(msgformat)
            comment = "Output: " + msgformat
        else:
            param_types = [t for name, t in msgproto.lookup_params(msgformat)]
            comment = msgformat
        params = "0"
        types = tuple([t.__class__.__name__ for t in param_types])
        if types:
            paramid = self.all_param_types.get(types)
            if paramid is None:
                paramid = len(self.all_param_types)
                self.all_param_types[types] = paramid
            params = "command_parameters%d" % (paramid,)
        out = """
    // %s
    .encoded_msgid=%d, // msgid=%d
    .num_params=%d,
    .param_types = %s,
""" % (
            comment,
            encoded_msgid,
            self.encid_to_msgid[encoded_msgid],
            len(types),
            params,
        )
        if msgtype == "response":
            num_args = (
                len(types)
                + types.count("PT_progmem_buffer")
                + types.count("PT_buffer")
            )
            out += "    .num_args=%d," % (num_args,)
        else:
            msgid_size = 1
            if encoded_msgid >= 0x80:
                msgid_size = 2
            max_size = min(
                msgproto.MESSAGE_MAX,
                (
                    msgproto.MESSAGE_MIN
                    + msgid_size
                    + sum([t.max_length for t in param_types])
                ),
            )
            out += "    .max_size=%d," % (max_size,)
        return out

    def generate_responses_code(self):
        encoder_defs = []
        output_code = []
        encoder_code = []
        did_output = {}
        for msgname, msg in self.encoders:
            encoded_msgid = self.msg_to_encid[msg]
            if encoded_msgid in did_output:
                continue
            did_output[encoded_msgid] = True
            code = (
                '    if (__builtin_strcmp(str, "%s") == 0)\n'
                "        return &command_encoder_%s;\n" % (msg, encoded_msgid)
            )
            if msgname is None:
                parsercode = self.build_parser(encoded_msgid, msg, "output")
                output_code.append(code)
            else:
                parsercode = self.build_parser(encoded_msgid, msg, "command")
                encoder_code.append(code)
            encoder_defs.append(
                "const struct command_encoder command_encoder_%s PROGMEM = {"
                "    %s\n};\n" % (encoded_msgid, parsercode)
            )
        fmt = """
%s

const __always_inline struct command_encoder *
ctr_lookup_encoder(const char *str)
{
    %s
    return NULL;
}

const __always_inline struct command_encoder *
ctr_lookup_output(const char *str)
{
    %s
    return NULL;
}
"""
        return fmt % (
            "".join(encoder_defs).strip(),
            "".join(encoder_code).strip(),
            "".join(output_code).strip(),
        )

    def generate_commands_code(self):
        cmd_by_encid = {
            self.msg_to_encid[self.messages_by_name.get(msgname, msgname)]: cmd
            for msgname, cmd in self.commands.items()
        }
        max_cmd_encid = max(cmd_by_encid.keys())
        index = []
        externs = {}
        for encoded_msgid in range(max_cmd_encid + 1):
            if encoded_msgid not in cmd_by_encid:
                index.append(" {\n},")
                continue
            funcname, flags, msgname = cmd_by_encid[encoded_msgid]
            msg = self.messages_by_name[msgname]
            externs[funcname] = 1
            parsercode = self.build_parser(encoded_msgid, msg, "response")
            index.append(
                " {%s\n    .flags=%s,\n    .func=%s\n},"
                % (parsercode, flags, funcname)
            )
        index = "".join(index).strip()
        externs = "\n".join(
            [
                "extern void " + funcname + "(uint32_t*);"
                for funcname in sorted(externs)
            ]
        )
        fmt = """
%s

const struct command_parser command_index[] PROGMEM = {
%s
};

const uint16_t command_index_size PROGMEM = ARRAY_SIZE(command_index);
"""
        return fmt % (externs, index)

    def generate_param_code(self):
        sorted_param_types = sorted(
            [(i, a) for a, i in self.all_param_types.items()]
        )
        params = [""]
        for paramid, argtypes in sorted_param_types:
            params.append(
                "static const uint8_t command_parameters%d[] PROGMEM = {\n"
                "    %s };"
                % (
                    paramid,
                    ", ".join(argtypes),
                )
            )
        params.append("")
        return "\n".join(params)

    def generate_code(self, options):
        self.create_message_ids()
        parsercode = self.generate_responses_code()
        cmdcode = self.generate_commands_code()
        paramcode = self.generate_param_code()
        return paramcode + parsercode + cmdcode


Handlers.append(HandleCommandGeneration())


######################################################################
# Version generation
######################################################################


# Run program and return the specified output
def check_output(prog):
    logging.debug("Running %s" % (repr(prog),))
    try:
        process = subprocess.Popen(shlex.split(prog), stdout=subprocess.PIPE)
        output = process.communicate()[0]
        retcode = process.poll()
    except OSError:
        logging.debug("Exception on run: %s" % (traceback.format_exc(),))
        return ""
    logging.debug("Got (code=%s): %s" % (retcode, repr(output)))
    if retcode:
        return ""
    try:
        return str(output.decode("utf8"))
    except UnicodeError:
        logging.debug("Exception on decode: %s" % (traceback.format_exc(),))
        return ""


# Obtain version info from "git" program
def git_version():
    if not os.path.exists(".git"):
        logging.debug("No '.git' file/directory found")
        return ""
    ver = check_output("git describe --always --tags --long --dirty").strip()
    logging.debug("Got git version: %s" % (repr(ver),))
    return ver


def build_version(extra, cleanbuild):
    version = git_version()
    if not version:
        cleanbuild = False
        version = "?"
    elif "dirty" in version:
        cleanbuild = False
    if not cleanbuild:
        btime = time.strftime("%Y%m%d_%H%M%S")
        hostname = socket.gethostname()
        version = "%s-%s-%s" % (version, btime, hostname)
    return version + extra


# Run "tool --version" for each specified tool and extract versions
def tool_versions(tools):
    tools = [t.strip() for t in tools.split(";")]
    versions = ["", ""]
    success = 0
    for tool in tools:
        # Extract first line from "tool --version" output
        verstr = check_output("%s --version" % (tool,)).split("\n")[0]
        # Check if this tool looks like a binutils program
        isbinutils = 0
        if verstr.startswith("GNU "):
            isbinutils = 1
            verstr = verstr[4:]
        # Extract version information and exclude program name
        if " " not in verstr:
            continue
        prog, ver = verstr.split(" ", 1)
        if not prog or not ver:
            continue
        # Check for any version conflicts
        if versions[isbinutils] and versions[isbinutils] != ver:
            logging.debug(
                "Mixed version %s vs %s"
                % (repr(versions[isbinutils]), repr(ver))
            )
            versions[isbinutils] = "mixed"
            continue
        versions[isbinutils] = ver
        success += 1
    cleanbuild = versions[0] and versions[1] and success == len(tools)
    return cleanbuild, "gcc: %s binutils: %s" % (versions[0], versions[1])


# Add version information to the data dictionary
class HandleVersions:
    def __init__(self):
        self.ctr_dispatch = {}
        self.toolstr = self.version = ""

    def update_data_dictionary(self, data):
        data["version"] = self.version
        data["build_versions"] = self.toolstr
        data["app"] = "Danger-Klipper"
        data["license"] = "GNU GPLv3"

    def generate_code(self, options):
        cleanbuild, self.toolstr = tool_versions(options.tools)
        self.version = build_version(options.extra, cleanbuild)
        sys.stdout.write("Version: %s\n" % (self.version,))
        return "\n// version: %s\n// build_versions: %s\n" % (
            self.version,
            self.toolstr,
        )


Handlers.append(HandleVersions())


######################################################################
# Identify data dictionary generation
######################################################################


# Automatically generate the wire protocol data dictionary
class HandleIdentify:
    def __init__(self):
        self.ctr_dispatch = {}

    def update_data_dictionary(self, data):
        pass

    def generate_code(self, options):
        # Generate data dictionary
        data = {}
        for h in Handlers:
            h.update_data_dictionary(data)
        datadict = json.dumps(data, separators=(",", ":"), sort_keys=True)

        # Write data dictionary
        if options.write_dictionary:
            f = open(options.write_dictionary, "w")
            f.write(datadict)
            f.close()

        # Format compressed info into C code
        zdatadict = bytearray(zlib.compress(datadict.encode(), 9))
        out = []
        for i in range(len(zdatadict)):
            if i % 8 == 0:
                out.append("\n   ")
            out.append(" 0x%02x," % (zdatadict[i],))
        fmt = """
const uint8_t command_identify_data[] PROGMEM = {%s
};

// Identify size = %d (%d uncompressed)
const uint32_t command_identify_size PROGMEM
    = ARRAY_SIZE(command_identify_data);
"""
        return fmt % ("".join(out), len(zdatadict), len(datadict))


Handlers.append(HandleIdentify())


######################################################################
# Main code
######################################################################


def main():
    usage = "%prog [options] <cmd section file> <output.c>"
    opts = optparse.OptionParser(usage)
    opts.add_option(
        "-e",
        "--extra",
        dest="extra",
        default="",
        help="extra version string to append to version",
    )
    opts.add_option(
        "-d",
        dest="write_dictionary",
        help="file to write mcu protocol dictionary",
    )
    opts.add_option(
        "-t",
        "--tools",
        dest="tools",
        default="",
        help="list of build programs to extract version from",
    )
    opts.add_option(
        "-v", action="store_true", dest="verbose", help="enable debug messages"
    )

    options, args = opts.parse_args()
    if len(args) != 2:
        opts.error("Incorrect arguments")
    incmdfile, outcfile = args
    if options.verbose:
        logging.basicConfig(level=logging.DEBUG)

    # Parse request file
    ctr_dispatch = {k: v for h in Handlers for k, v in h.ctr_dispatch.items()}
    f = open(incmdfile, "r")
    data = f.read()
    f.close()
    for req in data.split("\n"):
        req = req.lstrip()
        if not req:
            continue
        cmd = req.split()[0]
        if cmd not in ctr_dispatch:
            error("Unknown build time command '%s'" % cmd)
        ctr_dispatch[cmd](req)

    # Write output
    code = "".join([FILEHEADER] + [h.generate_code(options) for h in Handlers])
    f = open(outcfile, "w")
    f.write(code)
    f.close()


if __name__ == "__main__":
    main()
