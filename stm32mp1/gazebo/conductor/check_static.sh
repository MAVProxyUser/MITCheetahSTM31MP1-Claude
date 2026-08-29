#!/bin/bash
# Syntax-check the panel's JS before trusting it. A single SyntaxError in
# app.js does not degrade the page - it kills EVERY handler, so the panel
# renders its initial HTML and then freezes: no slots, no log, no fleet
# status, a stale caption. That is exactly what one shadowed `const cap`
# did on 2026-08-29, and it looked from the outside like a dead conductor.
# Serving the file and grepping it for the new code (which is what was
# actually done, and passed) proves nothing about whether it PARSES.
cd "$(dirname "$0")"
fail=0
for f in static/*.js; do
  if node --check "$f" 2>/dev/null; then echo "ok    $f"; else
    echo "FAIL  $f"; node --check "$f" 2>&1 | head -5; fail=1; fi
done
exit $fail
