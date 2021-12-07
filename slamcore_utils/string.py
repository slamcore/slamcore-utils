import random
import string


def get_rand_string(len=10) -> str:
    """Return a random string containing ascii characters, digits and/or punctuation marks."""
    return "".join(random.choice(string.ascii_letters + string.digits) for _ in range(len))
