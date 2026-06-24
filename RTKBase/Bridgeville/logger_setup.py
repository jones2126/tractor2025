import logging
import logging.handlers
import os

def logger_setup(log_filename="rtcm_server.log", max_file_size=5*1024*1024, backup_count=3, log_level=logging.INFO):
    """
    Set up a logger with file rotation.
    
    Args:
        log_filename: Name of the log file
        max_file_size: Maximum size of each log file in bytes
        backup_count: Number of backup files to keep
        log_level: Logging level
    
    Returns:
        Logger instance
    """
    # Create logs directory if it doesn't exist
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Full path to log file
    log_path = os.path.join(log_dir, log_filename)
    
    # Create logger
    logger = logging.getLogger(__name__)
    logger.setLevel(log_level)
    
    # Create rotating file handler
    handler = logging.handlers.RotatingFileHandler(
        log_path, 
        maxBytes=max_file_size, 
        backupCount=backup_count
    )
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    handler.setFormatter(formatter)
    
    # Add handler to logger
    logger.addHandler(handler)
    
    return logger
