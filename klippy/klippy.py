#!/usr/bin/env python3

import sys, pathlib

if __name__ == "__main__":
    sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))

    from klippy.printer import main

    main()
