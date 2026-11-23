"""Shared helper of the variants: replace a function of the main code, and put it back."""


def replace(owner, name, new):
    """Set owner.name = new and return the function that restores the original. Raises
    AttributeError if the main code no longer has owner.name: the variant must be updated."""
    if not hasattr(owner, name):
        raise AttributeError(f"{getattr(owner, '__name__', owner)}.{name} is not in the main code any "
                             f"more: this variant must be updated")
    old = getattr(owner, name)
    setattr(owner, name, new)
    return lambda: setattr(owner, name, old)


def chain(*restores):
    """One restore function for several replacements."""
    def restore():
        for r in reversed(restores):
            r()
    return restore
