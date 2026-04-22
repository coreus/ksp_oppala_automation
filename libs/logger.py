import logging
from rich.logging import RichHandler


def setup(level=logging.DEBUG):
    logging.basicConfig(
        level=level,
        format="%(message)s",
        datefmt="[%X]",
        handlers=[
            RichHandler(
                rich_tracebacks=True,
                show_path=True,
                markup=True,
                log_time_format="[%H:%M:%S]",
            )
        ],
    )
