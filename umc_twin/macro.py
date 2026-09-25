"""Haas / Fanuc macro-B subset: variables, expressions and control statements.

Supported
  variables     #1-#33 local (per G65 call), #100-#199 and #500-#999 common, #0 = vacant
  expressions   + - * / MOD, [ ] grouping, SIN COS TAN ASIN ACOS ATAN SQRT ABS ROUND FIX FUP LN EXP,
                EQ NE GT LT GE LE AND OR XOR, indirect #[expr]
  statements    #n = expr, GOTO n, IF [cond] GOTO n, IF [cond] THEN #n = expr, WHILE [cond] DOm ... ENDm
  words         any address can take an expression: X#24, Z-[#26+#18], F#9. A word whose value is
                vacant is dropped, as on the control.
  system        read/write by the interpreter through `system_get` / `system_set` (positions,
                work offsets, tool offsets -- see gcode.Interpreter._sysvar)

Angles are in degrees, like the control. Vacant (#0 / never set) reads as None and counts as 0 in
arithmetic.
"""
from __future__ import annotations

import math
import re
from typing import Callable

FUNCS = {
    "SIN": lambda v: math.sin(math.radians(v)), "COS": lambda v: math.cos(math.radians(v)),
    "TAN": lambda v: math.tan(math.radians(v)), "ASIN": lambda v: math.degrees(math.asin(v)),
    "ACOS": lambda v: math.degrees(math.acos(v)), "ATAN": lambda v: math.degrees(math.atan(v)),
    "SQRT": math.sqrt, "ABS": abs, "ROUND": lambda v: float(math.floor(v + 0.5)) if v >= 0 else -float(math.floor(-v + 0.5)),
    "FIX": lambda v: float(math.floor(v)), "FUP": lambda v: float(math.ceil(v)),
    "LN": math.log, "EXP": math.exp,
}
COMPARE = {"EQ": lambda a, b: a == b, "NE": lambda a, b: a != b, "GT": lambda a, b: a > b,
           "LT": lambda a, b: a < b, "GE": lambda a, b: a >= b, "LE": lambda a, b: a <= b}
TOKEN_RE = re.compile(r"\s*(?:(\d+\.?\d*|\.\d+)|([A-Z]+)|(.))")


class MacroError(Exception):
    pass


class Variables:
    """Macro variables with a stack of local (#1-#33) frames for G65 calls."""

    def __init__(self, system_get: Callable[[int], float | None] | None = None,
                 system_set: Callable[[int, float | None], bool] | None = None):
        self.common: dict[int, float | None] = {}
        self.locals: list[dict[int, float | None]] = [{}]
        self.system_get = system_get
        self.system_set = system_set

    def get(self, n: int) -> float | None:
        if n == 0:
            return None
        if 1 <= n <= 33:
            return self.locals[-1].get(n)
        if 100 <= n <= 199 or 500 <= n <= 999:
            return self.common.get(n)
        if self.system_get:
            return self.system_get(n)
        return None

    def set(self, n: int, v: float | None):
        if n == 0:
            raise MacroError("#0 is read-only")
        if 1 <= n <= 33:
            self.locals[-1][n] = v
        elif 100 <= n <= 199 or 500 <= n <= 999:
            self.common[n] = v
        elif not (self.system_set and self.system_set(n, v)):
            raise MacroError(f"#{n} can't be written")

    def push(self, args: dict[int, float]):
        self.locals.append(dict(args))

    def pop(self):
        if len(self.locals) > 1:
            self.locals.pop()


class _Parser:
    def __init__(self, text: str, vars: Variables):
        self.toks = [(num, word, ch) for num, word, ch in TOKEN_RE.findall(text.upper()) if num or word or ch.strip()]
        self.i = 0
        self.vars = vars

    def peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else ("", "", "")

    def take(self):
        t = self.peek()
        self.i += 1
        return t

    def expect(self, ch):
        t = self.take()
        if t[2] != ch:
            raise MacroError(f"expected {ch!r}")

    def done(self):
        return self.i >= len(self.toks)

    # grammar: or -> cmp (OR|XOR|AND cmp)* ; cmp -> add (EQ.. add)? ; add -> mul (+|- mul)* ;
    #          mul -> unary (*|/|MOD unary)* ; unary -> (+|-) unary | atom
    def expr(self):
        v = self.cmp()
        while self.peek()[1] in ("OR", "XOR", "AND"):
            op = self.take()[1]
            b = self.cmp()
            a, b2 = bool(v), bool(b)
            v = float(a or b2) if op == "OR" else float(a != b2) if op == "XOR" else float(a and b2)
        return v

    def cmp(self):
        v = self.add()
        if self.peek()[1] in COMPARE:
            op = self.take()[1]
            v = float(COMPARE[op](_n(v), _n(self.add())))
        return v

    def add(self):
        v = self.mul()
        while self.peek()[2] in ("+", "-"):
            op = self.take()[2]
            b = self.mul()
            v = _n(v) + _n(b) if op == "+" else _n(v) - _n(b)
        return v

    def mul(self):
        v = self.unary()
        while self.peek()[2] in ("*", "/") or self.peek()[1] == "MOD":
            t = self.take()
            b = _n(self.unary())
            if t[2] == "*":
                v = _n(v) * b
            elif t[2] == "/":
                if b == 0:
                    raise MacroError("division by zero")
                v = _n(v) / b
            else:
                v = math.fmod(_n(v), b)
        return v

    def unary(self):
        if self.peek()[2] in ("+", "-"):
            sign = -1.0 if self.take()[2] == "-" else 1.0
            return sign * _n(self.unary())
        return self.atom()

    def atom(self):
        num, word, ch = self.take()
        if num:
            return float(num)
        if ch == "#":
            return self.vars.get(self.var_index())
        if ch == "[":
            v = self.expr()
            self.expect("]")
            return v
        if word in FUNCS:
            self.expect("[")
            v = self.expr()
            if word == "ATAN" and self.peek()[2] == "/":  # ATAN[a]/[b]
                self.expect("]")
                self.take()
                self.expect("[")
                b = self.expr()
                self.expect("]")
                return math.degrees(math.atan2(_n(v), _n(b)))
            self.expect("]")
            try:
                return FUNCS[word](_n(v))
            except ValueError as e:
                raise MacroError(f"{word}[{v}]: {e}") from None
        raise MacroError(f"unexpected {num or word or ch!r}")

    def var_index(self) -> int:
        num, word, ch = self.peek()
        if num:
            self.take()
            return int(float(num))
        if ch == "[":
            self.take()
            v = self.expr()
            self.expect("]")
            return int(_n(v))
        raise MacroError("bad variable reference")


def _n(v) -> float:
    return 0.0 if v is None else float(v)


def evaluate(text: str, vars: Variables) -> float | None:
    p = _Parser(text, vars)
    v = p.expr()
    if not p.done():
        raise MacroError(f"trailing text in expression {text!r}")
    return v


ASSIGN_RE = re.compile(r"^#\s*(\d+|\[.*?\])\s*=\s*(.+)$")
IF_GOTO_RE = re.compile(r"^IF\s*(\[.*\])\s*GOTO\s*(.+)$")
IF_THEN_RE = re.compile(r"^IF\s*(\[.*\])\s*THEN\s*(.+)$")
GOTO_RE = re.compile(r"^GOTO\s*(.+)$")
WHILE_RE = re.compile(r"^WHILE\s*(\[.*\])\s*DO\s*(\d+)$")
END_RE = re.compile(r"^END\s*(\d+)$")
DO_RE = re.compile(r"^DO\s*(\d+)$")


def is_macro_line(line: str) -> bool:
    return "#" in line or "[" in line or line.startswith(("IF", "GOTO", "WHILE", "END", "DO"))


def assign(stmt: str, vars: Variables) -> bool:
    m = ASSIGN_RE.match(stmt)
    if not m:
        return False
    target = m.group(1)
    n = int(float(target)) if not target.startswith("[") else int(_n(evaluate(target, vars)))
    vars.set(n, evaluate(m.group(2), vars))
    return True


def substitute(line: str, vars: Variables) -> str:
    """Replace every address value that is an expression with its number: 'X#24 Z-[#1+2]' ->
    'X12.5 Z-3.0'. Words whose value is vacant are dropped."""
    out, i, n = [], 0, len(line)
    while i < n:
        ch = line[i]
        if ch.isalpha():
            j = i + 1
            while j < n and line[j] == " ":
                j += 1
            k = j
            sign = ""
            if k < n and line[k] in "+-":
                sign, k = line[k], k + 1
            if k < n and line[k] in "#[":
                end = _expr_end(line, k)
                val = evaluate(line[k:end], vars)
                if val is not None:
                    val = -val if sign == "-" else val
                    out.append(f"{ch}{_fmt(val)} ")
                i = end
                continue
        out.append(ch)
        i += 1
    return "".join(out)


def _expr_end(s: str, k: int) -> int:
    """End index of a #var / #[..] / [..] starting at k."""
    if s[k] == "[":
        return _match_bracket(s, k) + 1
    k += 1  # '#'
    while k < len(s) and s[k] == " ":
        k += 1
    if k < len(s) and s[k] == "[":
        return _match_bracket(s, k) + 1
    while k < len(s) and (s[k].isdigit() or s[k] == "."):
        k += 1
    return k


def _match_bracket(s: str, k: int) -> int:
    depth = 0
    for i in range(k, len(s)):
        if s[i] == "[":
            depth += 1
        elif s[i] == "]":
            depth -= 1
            if depth == 0:
                return i
    raise MacroError("unbalanced [ ]")


def _fmt(v: float) -> str:
    s = f"{v:.6f}".rstrip("0")
    return s if "." in s else s + "."


# G65 argument letters -> local variables (argument specification I)
G65_ARGS = {"A": 1, "B": 2, "C": 3, "I": 4, "J": 5, "K": 6, "D": 7, "E": 8, "F": 9, "H": 11, "M": 13,
            "Q": 17, "R": 18, "S": 19, "T": 20, "U": 21, "V": 22, "W": 23, "X": 24, "Y": 25, "Z": 26}
