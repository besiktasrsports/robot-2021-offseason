# robot-2021-offseason

[![Formatting (pre-commit)](https://github.com/sneakysnakesfrc/robot-2021-offseason/actions/workflows/format.yaml/badge.svg?branch=main)](https://github.com/sneakysnakesfrc/robot-2021-offseason/actions/workflows/format.yaml?query=branch%3Amain)
[![Build And Test](https://github.com/sneakysnakesfrc/robot-2021-offseason/actions/workflows/build_and_test.yaml/badge.svg?branch=main)](https://github.com/sneakysnakesfrc/robot-2021-offseason/actions/workflows/build_and_test.yaml?query=branch%3Amain)

**Formatting with pre-commit**

pre-commit is a tool that is used in this repository to check and apply style guidelines automatically. To install pre-commit into your system:

    pip3 install pre-commit

Then under root directory install the git hooks like this:

    pre-commit install

With this pre-commit will automatically run and check a list of styling including clang-format, end of files and trailing whitespaces whenever you run `git commit`. To run pre-commit any time other than `git commit`:

    pre-commit run -a
