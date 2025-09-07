#!/usr/bin/env bash

# lint.sh
set -euo pipefail

# Find all targets to lint
targets=$(bazel query 'kind(".*(cc_library|cc_binary|cc_test)", //...)')

# Run the lint aspect on the targets
bazel build \
    --aspects=//tools/lint:linters.bzl%clang_tidy \
    --@aspect_rules_lint//lint:fail_on_violation \
    --keep_going \
    $targets
