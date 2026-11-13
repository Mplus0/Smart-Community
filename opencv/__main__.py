"""从 Smart-Community 仓库根目录运行 python -m opencv。"""

import sys

from .src.cli import main


if __name__ == "__main__":
    sys.exit(main())
