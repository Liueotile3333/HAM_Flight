#!/usr/bin/env python3

import argparse as ap
import argcomplete


def main(**args):
    del args


if __name__ == '__main__':
    parser = ap.ArgumentParser()
    parser.add_argument('positional', choices=['spam', 'eggs'])
    parser.add_argument('--optional', choices=['foo1', 'foo2', 'bar'])
    argcomplete.autocomplete(parser)
    parsed_args = parser.parse_args()
    main(**vars(parsed_args))
