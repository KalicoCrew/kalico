# Low level unix utility functions
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import fcntl
import json
import logging
import multiprocessing
import os
import pickle
import pty
import signal
import subprocess
import termios
import time
import traceback

from . import queuelogger

######################################################################
# Low-level Unix commands
######################################################################


# Return the SIGINT interrupt handler back to the OS default
def fix_sigint():
    signal.signal(signal.SIGINT, signal.SIG_DFL)


fix_sigint()


# Set a file-descriptor as non-blocking
def set_nonblock(fd):
    fcntl.fcntl(
        fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK
    )


# Clear HUPCL flag
def clear_hupcl(fd):
    attrs = termios.tcgetattr(fd)
    attrs[2] = attrs[2] & ~termios.HUPCL
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
    except termios.error:
        pass


# Support for creating a pseudo-tty for emulating a serial port
def create_pty(ptyname):
    mfd, sfd = pty.openpty()
    try:
        os.unlink(ptyname)
    except os.error:
        pass
    filename = os.ttyname(sfd)
    os.chmod(filename, 0o660)
    os.symlink(filename, ptyname)
    set_nonblock(mfd)
    old = termios.tcgetattr(mfd)
    old[3] = old[3] & ~termios.ECHO
    termios.tcsetattr(mfd, termios.TCSADRAIN, old)
    return mfd


######################################################################
# Background processes
######################################################################


class BackgroundError(Exception):
    pass


class BackgroundTimeout(BackgroundError):
    pass


_EXITED = object()
_TIMED_OUT = object()


def run_in_background(
    printer,
    func,
    *,
    timeout=None,
    progress_msg=None,
    progress_interval=5.0,
    interrupt_token=None,
):
    """Run func() in a forked process and return its result.

    The reactor keeps running while this waits, so slow calculations or
    imports do not stall MCU communication or timers. The result must be
    picklable.

    Raises BackgroundError with the child's traceback if func raises or
    the process exits without a result. Raises BackgroundTimeout, after
    killing the process, if timeout seconds pass first. When progress_msg
    is set it is sent to the G-code terminal every progress_interval
    seconds.

    If interrupt_token (by default the G-code interrupt token current at
    the call) is interrupted, for example by HEATER_INTERRUPT or a
    shutdown, the process is killed and the token's error, or a
    WaitInterruption, is raised.
    """
    reactor = printer.get_reactor()
    gcode = printer.lookup_object("gcode")
    if interrupt_token is None:
        interrupt_token = gcode.get_interrupt_token()
    # func is usually a closure, which only a forked child can run.
    ctx = multiprocessing.get_context("fork")
    read_fd, write_fd = os.pipe()

    def child():
        queuelogger.clear_bg_logging()
        os.close(read_fd)
        try:
            data = pickle.dumps((False, func()))
        except Exception:
            data = pickle.dumps((True, traceback.format_exc()))
        with os.fdopen(write_fd, "wb") as f:
            f.write(data)

    proc = ctx.Process(target=child, daemon=True)
    proc.start()
    # Without the parent's copy of the write end, the pipe reports EOF as
    # soon as the child exits, whether or not it wrote a result.
    os.close(write_fd)
    set_nonblock(read_fd)
    chunks = []
    done = reactor.completion()

    def read_result(eventtime):
        try:
            data = os.read(read_fd, 65536)
        except BlockingIOError:
            return
        except OSError:
            data = b""
        if data:
            chunks.append(data)
        elif not done.test():
            done.complete(_EXITED)

    def interrupted(result):
        if not done.test():
            done.complete(result)

    def report_progress(eventtime):
        gcode.respond_info(progress_msg, log=False)
        return eventtime + progress_interval

    start_time = reactor.monotonic()
    fd_handler = reactor.register_fd(read_fd, read_result)
    interrupt_token.add_callback(interrupted)
    progress_timer = None
    if progress_msg:
        progress_timer = reactor.register_timer(
            report_progress, start_time + progress_interval
        )
    deadline = reactor.NEVER if timeout is None else start_time + timeout
    try:
        outcome = done.wait(deadline, _TIMED_OUT)
    finally:
        interrupt_token.remove_callback(interrupted)
        if progress_timer is not None:
            reactor.unregister_timer(progress_timer)
        reactor.unregister_fd(fd_handler)
        os.close(read_fd)
        if done.result is not _EXITED:
            proc.kill()
        proc.join()
    if outcome is _TIMED_OUT:
        raise BackgroundTimeout(f"Did not finish within {timeout:.0f} seconds")
    if outcome is not _EXITED:
        from .gcode import WaitInterruption

        if isinstance(outcome, Exception):
            raise outcome
        raise WaitInterruption("Command interrupted")
    try:
        is_err, res = pickle.loads(b"".join(chunks))
    except Exception:
        raise BackgroundError(
            f"Process exited without a result (exit code {proc.exitcode})"
        )
    if is_err:
        raise BackgroundError(res)
    return res


######################################################################
# Helper code for extracting mcu build info
######################################################################


def dump_file_stats(build_dir, filename):
    fname = os.path.join(build_dir, filename)
    try:
        mtime = os.path.getmtime(fname)
        fsize = os.path.getsize(fname)
        timestr = time.asctime(time.localtime(mtime))
        logging.info("Build file %s(%d): %s", fname, fsize, timestr)
    except:
        logging.info("No build file %s", fname)


# Try to log information on the last mcu build
def dump_mcu_build():
    build_dir = os.path.join(os.path.dirname(__file__), "..")
    # Try to log last mcu config
    dump_file_stats(build_dir, ".config")
    try:
        f = open(os.path.join(build_dir, ".config"), "r")
        data = f.read(32 * 1024)
        f.close()
        logging.info(
            "========= Last MCU build config =========\n%s"
            "=======================",
            data,
        )
    except:
        pass
    # Try to log last mcu build version
    dump_file_stats(build_dir, "out/klipper.dict")
    try:
        f = open(os.path.join(build_dir, "out/klipper.dict"), "r")
        data = f.read(32 * 1024)
        f.close()
        data = json.loads(data)
        logging.info("Last MCU build version: %s", data.get("version", ""))
        logging.info("Last MCU build tools: %s", data.get("build_versions", ""))
        cparts = ["%s=%s" % (k, v) for k, v in data.get("config", {}).items()]
        logging.info("Last MCU build config: %s", " ".join(cparts))
    except:
        pass
    dump_file_stats(build_dir, "out/klipper.elf")


######################################################################
# General system and software information
######################################################################


def get_cpu_info():
    try:
        f = open("/proc/cpuinfo", "r")
        data = f.read()
        f.close()
    except (IOError, OSError) as e:
        logging.debug(
            "Exception on read /proc/cpuinfo: %s", traceback.format_exc()
        )
        return "?"
    lines = [l.split(":", 1) for l in data.split("\n")]
    lines = [(l[0].strip(), l[1].strip()) for l in lines if len(l) == 2]
    core_count = [k for k, v in lines].count("processor")
    model_name = dict(lines).get("model name", "?")
    return "%d core %s" % (core_count, model_name)


def get_version_from_file(klippy_src):
    try:
        with open(os.path.join(klippy_src, ".version")) as h:
            return h.read().rstrip()
    except IOError:
        pass
    return "?"


def _get_repo_info(gitdir):
    repo_info = {"branch": "?", "remote": "?", "url": "?"}
    prog_branch = ("git", "-C", gitdir, "branch", "--no-color")
    try:
        process = subprocess.Popen(
            prog_branch, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        branch_list, err = process.communicate()
        retcode = process.wait()
        if retcode != 0:
            logging.debug("Error running git branch: %s", err)
            return repo_info
        lines = str(branch_list.strip().decode()).split("\n")
        for line in lines:
            if line[0] == "*":
                repo_info["branch"] = line[1:].strip()
                break
        else:
            logging.debug("Unable to find current branch:\n%s", branch_list)
            return repo_info
        if repo_info["branch"].startswith("(HEAD detached"):
            parts = repo_info["branch"].strip("()").split()[-1].split("/", 1)
            if len(parts) != 2:
                return repo_info
            repo_info["remote"] = parts[0]
        else:
            key = "branch.%s.remote" % (repo_info["branch"],)
            prog_config = ("git", "-C", gitdir, "config", "--get", key)
            process = subprocess.Popen(
                prog_config, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            remote_info, err = process.communicate()
            retcode = process.wait()
            if retcode != 0:
                logging.debug("Error running git config: %s", err)
                return repo_info
            repo_info["remote"] = str(remote_info.strip().decode())
        prog_remote_url = (
            "git",
            "-C",
            gitdir,
            "remote",
            "get-url",
            repo_info["remote"],
        )
        process = subprocess.Popen(
            prog_remote_url, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        remote_url, err = process.communicate()
        retcode = process.wait()
        if retcode != 0:
            logging.debug("Error running git remote get-url: %s", err)
            return repo_info
        repo_info["url"] = str(remote_url.strip().decode())
    except:
        logging.debug("Error fetching repo info: %s", traceback.format_exc())
    return repo_info


def get_git_version(from_file=True):
    git_info = {
        "version": "?",
        "file_status": [],
        "branch": "?",
        "remote": "?",
        "url": "?",
    }
    klippy_src = os.path.dirname(__file__)

    # Obtain version info from "git" program
    gitdir = os.path.join(klippy_src, "..")
    prog_desc = (
        "git",
        "-C",
        gitdir,
        "describe",
        "--always",
        "--tags",
        "--long",
        "--dirty",
    )
    prog_status = ("git", "-C", gitdir, "status", "--porcelain", "--ignored")
    try:
        process = subprocess.Popen(
            prog_desc, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        ver, err = process.communicate()
        retcode = process.wait()
        if retcode == 0:
            git_info["version"] = str(ver.strip().decode())
            process = subprocess.Popen(
                prog_status, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            stat, err = process.communicate()
            status = [
                l.split(None, 1) for l in str(stat.strip().decode()).split("\n")
            ]
            retcode = process.wait()
            if retcode == 0:
                git_info["file_status"] = status
            else:
                logging.debug("Error getting git status: %s", err)
            git_info.update(_get_repo_info(gitdir))
            return git_info
        else:
            logging.debug("Error getting git version: %s", err)
    except:
        logging.debug("Exception on run: %s", traceback.format_exc())

    if from_file:
        git_info["version"] = get_version_from_file(klippy_src)
    return git_info
