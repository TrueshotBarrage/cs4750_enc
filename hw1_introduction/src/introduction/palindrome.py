def is_palindrome(n):
    assert(isinstance(n, int), "Did you cast to int in Q1.2?")
    """Return whether given integer is a palindrome.

    >>> is_palindrome(9669669)
    True
    >>> is_palindrome(16)
    False

    """
    # BEGIN QUESTION 1.1
    "*** REPLACE THIS LINE ***"
    return str(n) == str(n)[::-1]
    # END QUESTION 1.1
