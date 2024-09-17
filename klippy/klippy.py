#!/usr/bin/env python3

import sys, os

if __name__ == '__main__':
    sys.path.insert(0, os.path.abspath('.'))

    from klippy.printer import main

    main()
