from __future__ import annotations

import pytest
import pathlib
import subprocess
import sys
import tempfile
import shutil


def pytest_collect_file(parent, file_path):
    if file_path.suffix == ".test":
        return KlippyTest.from_parent(parent, path=file_path)


class KlippyTest(pytest.File):
    def relative_path(self, *parts, root=None):
        if not root:
            root = self.path.parent
        return root.joinpath(*parts).resolve()

    def collect(self):
        dict_path = pathlib.Path.cwd() / self.config.getoption("dictdir")

        with self.path.open("r", encoding="utf-8") as test_file:
            multi_test = False
            should_fail = False
            gcode = []
            config_file = None
            dictionaries = []

            for line in test_file:
                parts = line.strip().split()

                if not parts or line.strip().startswith("#"):
                    continue

                elif parts[0] == "SHOULD_FAIL":
                    should_fail = True

                elif parts[0] == "GCODE":
                    with self.relative_path(parts[1]).open(
                        "r", encoding="utf-8"
                    ) as fp:
                        gcode = fp.readlines()

                elif parts[0] == "DICTIONARY":
                    dictionaries = [
                        str(self.relative_path(parts[1], root=dict_path))
                    ]
                    for mcu_path in parts[2:]:
                        mcu, fname = mcu_path.split("=", maxsplit=1)
                        dictionaries.append(
                            f"{mcu}={self.relative_path(fname, root=dict_path)}"
                        )

                elif parts[0] == "CONFIG":
                    if config_file and not multi_test:
                        multi_test = True
                        yield KlippyTestItem.from_parent(
                            self,
                            name=str(config_file),
                            gcode=gcode,
                            config_file=config_file,
                            dictionaries=list(dictionaries),
                            should_fail=should_fail,
                        )

                    config_file = self.relative_path(parts[1])

                    if multi_test:
                        yield KlippyTestItem.from_parent(
                            self,
                            name=str(config_file),
                            gcode=gcode,
                            config_file=config_file,
                            dictionaries=list(dictionaries),
                            should_fail=should_fail,
                        )

                else:
                    gcode.append(line.strip())

            if not multi_test:
                yield KlippyTestItem.from_parent(
                    self,
                    name=str(config_file),
                    gcode=gcode,
                    config_file=config_file,
                    dictionaries=list(dictionaries),
                    should_fail=should_fail,
                )


class KlippyTestItem(pytest.Item):
    def __init__(
        self,
        *,
        config_file: pathlib.Path,
        dictionaries: list[str],
        gcode: list[str],
        should_fail: bool = False,
        **kwargs,
    ):
        super().__init__(**kwargs)

        self.config_file = config_file
        self.dictionaries = dictionaries
        self.gcode = gcode

        if should_fail:
            self.add_marker(pytest.mark.xfail)

    def setup(self):
        self.tmp_dir = pathlib.Path(tempfile.mkdtemp())

    def teardown(self):
        shutil.rmtree(self.tmp_dir, ignore_errors=True)

    def runtest(self):
        gcode_file = self.tmp_dir.joinpath("_test_.gcode")
        output_file = self.tmp_dir.joinpath("_test_.output")

        gcode_file.write_text("\n".join(self.gcode) + "\n")

        args = [sys.executable, "-m", "klippy", str(self.config_file)]
        args.extend(["-i", str(gcode_file)])
        args.extend(["-o", str(output_file)])
        args.extend(["-v"])

        for df in self.dictionaries:
            args.extend(["-d", df])

        subprocess.run(args, check=True, text=True, stderr=subprocess.STDOUT)

    def repr_failure(self, excinfo, style=None):
        if isinstance(excinfo.value, subprocess.CalledProcessError):
            return "\n".join(
                [
                    f"Error in {self.name}",
                    f"  Config File: {self.config_file}",
                    f"  Dictionaries: {', '.join(self.dictionaries)}",
                ]
            )

        return super().repr_failure(excinfo=excinfo, style=style)
