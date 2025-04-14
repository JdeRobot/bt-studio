class TooMuchAWSSpace(Exception):
    """Exception raised for using too much AWS space."""

    def __init__(self):
        self.message = "User is using too much AWS space"
        super().__init__(self.message)
        self.error_code = 507

    def __str__(self):
        return f"{self.message}"


class LimitReached(Exception):
    """Exception raised when a limit is reached"""

    def __init__(self, msg):
        self.message = f"{msg} limit reached"
        super().__init__(self.message)
        self.error_code = 400

    def __str__(self):
        return f"{self.message}"


class ResourceAlreadyExists(Exception):
    """Exception raised for finding a resource that already exists."""

    def __init__(self, msg):
        self.message = f"{msg} already exists"
        super().__init__(self.message)
        self.error_code = 409

    def __str__(self):
        return f"{self.message}"


class ResourceNotExists(Exception):
    """Exception raised for not finding a resource."""

    def __init__(self, msg):
        self.message = f"{msg} does not exist"
        super().__init__(self.message)
        self.error_code = 404

    def __str__(self):
        return f"{self.message}"
