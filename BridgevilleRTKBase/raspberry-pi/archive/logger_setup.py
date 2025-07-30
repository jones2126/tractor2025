import logging
import logging.handlers
import os

def logger_setup(
    logger_name=__name__,
    log_level=logging.INFO,
    log_dir="logs",
    log_filename="gps_rtcm_server.log",
    max_file_size=10*1024*1024,  # 10MB
    backup_count=5,
    console_output=True,
    file_output=True
):
    """
    Set up logging with both file and console output.
    
    Args:
        logger_name (str): Name of the logger (default: __name__)
        log_level: Logging level (default: logging.INFO)
        log_dir (str): Directory for log files (default: "logs")
        log_filename (str): Log file name (default: "gps_rtcm_server.log")
        max_file_size (int): Max file size in bytes before rotation (default: 10MB)
        backup_count (int): Number of backup files to keep (default: 5)
        console_output (bool): Enable console logging (default: True)
        file_output (bool): Enable file logging (default: True)
    
    Returns:
        logging.Logger: Configured logger instance
    """
    
    # Create logs directory if it doesn't exist
    if file_output and not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Create logger
    logger = logging.getLogger(logger_name)
    logger.setLevel(log_level)
    
    # Clear any existing handlers to avoid duplicates
    logger.handlers.clear()
    
    # Create formatter
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    
    # File handler with rotation
    if file_output:
        file_handler = logging.handlers.RotatingFileHandler(
            filename=os.path.join(log_dir, log_filename),
            maxBytes=max_file_size,
            backupCount=backup_count
        )
        file_handler.setFormatter(formatter)
        file_handler.setLevel(log_level)
        logger.addHandler(file_handler)
    
    # Console handler
    if console_output:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        console_handler.setLevel(log_level)
        logger.addHandler(console_handler)
    
    return logger

# Example usage:
if __name__ == "__main__":
    # Basic setup - logs to both console and file
    logger = logger_setup()
    
    # Custom setup examples:
    
    # Console only
    # logger = logger_setup(file_output=False)
    
    # File only
    # logger = logger_setup(console_output=False)
    
    # Custom settings
    # logger = logger_setup(
    #     log_dir="my_logs",
    #     log_filename="custom.log",
    #     max_file_size=5*1024*1024,  # 5MB
    #     backup_count=3,
    #     log_level=logging.DEBUG
    # )
    
    # Test the logger
    logger.info("Logger setup complete")
    logger.warning("This is a warning message")
    logger.error("This is an error message")
