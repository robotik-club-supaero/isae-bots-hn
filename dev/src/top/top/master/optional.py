class Optional:
    """
    Wraps an object in a "best effort" manner.
    
    If the object fails to initialize, the error is silently ignored and future method calls on the object are silently replaced with a no-op.
    This does not support properties or methods that return a value.
    """


    def __init__(self, logger, fn, *args, **kwargs):
        """
        - logger: to log the error if initialization fails
        - fn: the function that attempts to created the object
        - *args: positional args to give to fn
        - **kwargs: keyword args to give to fn
        """
        try:
            self._value = fn(*args, **kwargs)
        except Exception as e:
            self._value = None
            logger.warn(f"Failed to initialize {fn}: {e}. Continuing anyway.")

    def _dummy_fn(self, *args, **kwargs):
        pass

    def __getattribute__(self, name):
        if name.startswith("_"):
            return super().__getattribute__(name)
        if self._value is not None:
            return self._value.__getattribute__(name)
        else:
            return self._dummy_fn