import logging
import coloredlogs
import os

def setup_logging():
    """
    Sets up logging for the application.
    - Creates a 'logs' directory if it doesn't exist.
    - Adds a file handler to save logs to a file.
    - Adds a stream handler to print colored logs to the console.
    """
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    log_file = os.path.join(log_dir, "air_defense.log")

    # Get the root logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    # Create handlers
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)

    # Create formatters and add it to handlers
    file_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(file_format)

    # Add handlers to the logger
    if not logger.handlers:
        logger.addHandler(file_handler)

    # Setup colored logs
    coloredlogs.install(
        level='DEBUG',
        logger=logger,
        fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        level_styles={
            'debug': {'color': 'green'},
            'info': {'color': 'cyan'},
            'warning': {'color': 'yellow'},
            'error': {'color': 'red'},
            'critical': {'color': 'red', 'bold': True}
        }
    )
