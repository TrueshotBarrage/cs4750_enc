#!/usr/bin/env python

import rosunit
import unittest

from introduction.palindrome import is_palindrome


class TestPalindrome(unittest.TestCase):
    def test_small(self):
        self.assertEqual(is_palindrome(0), True)
        self.assertEqual(is_palindrome(1), True)
        self.assertEqual(is_palindrome(10), False)

    def test_large(self):
        self.assertEqual(is_palindrome(112211), True)
        self.assertEqual(is_palindrome(13344331), True)
        self.assertEqual(is_palindrome(1334433), False)


if __name__ == "__main__":
    rosunit.unitrun("introduction", "test_palindrome", TestPalindrome)
