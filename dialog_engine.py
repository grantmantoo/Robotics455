import random
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


PUNCT_RE = re.compile(r"[.,!?]")
SPACE_RE = re.compile(r"\s+")
VAR_RE = re.compile(r"\$([A-Za-z_][A-Za-z0-9_]*)")
ACTION_RE = re.compile(r"<([A-Za-z_][A-Za-z0-9_]*)>")
RULE_RE = re.compile(r"^\s*u(\d*)\s*:\s*\((.*?)\)\s*:\s*(.+?)\s*$")
DEF_RE = re.compile(r"^\s*~([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.+?)\s*$")
INTERRUPT_WORDS = {"stop", "cancel", "reset", "quit"}


@dataclass
class ParseError:
    filename: str
    line: int
    category: str
    message: str
    fatal: bool = False

    def __str__(self) -> str:
        sev = "FATAL" if self.fatal else "NON-FATAL"
        return f"{self.filename}:{self.line} [{self.category}] [{sev}] {self.message}"


@dataclass
class Rule:
    level: int
    pattern: str
    output: str
    line: int
    children: List["Rule"] = field(default_factory=list)
    order: int = 0


def _strip_comments(line: str) -> str:
    in_quote = False
    for i, ch in enumerate(line):
        if ch == '"':
            in_quote = not in_quote
        if ch == "#" and not in_quote:
            return line[:i]
    return line


def normalize_text(text: str) -> str:
    text = text.lower()
    text = PUNCT_RE.sub(" ", text)
    text = SPACE_RE.sub(" ", text).strip()
    return text


def parse_choice_items(content: str) -> List[str]:
    items: List[str] = []
    i = 0
    n = len(content)
    while i < n:
        while i < n and content[i].isspace():
            i += 1
        if i >= n:
            break
        if content[i] == '"':
            i += 1
            j = i
            while j < n and content[j] != '"':
                j += 1
            if j >= n:
                raise ValueError("unclosed quote in choice")
            items.append(content[i:j])
            i = j + 1
        else:
            j = i
            while j < n and not content[j].isspace():
                j += 1
            items.append(content[i:j])
            i = j
    return [x for x in items if x]


def _find_matching_bracket(text: str, start: int) -> int:
    in_quote = False
    i = start + 1
    while i < len(text):
        ch = text[i]
        if ch == '"':
            in_quote = not in_quote
        elif ch == "]" and not in_quote:
            return i
        i += 1
    return -1


class DialogEngine:
    def __init__(self, filename: str, seed: Optional[int] = None):
        self.filename = filename
        self.seed = seed
        self.rng = random.Random(seed)
        self.definitions: Dict[str, List[str]] = {}
        self.top_rules: List[Rule] = []
        self.errors: List[ParseError] = []
        self.variables: Dict[str, str] = {}
        self.scope_stack: List[Rule] = []
        self.unmatched_in_scope = 0
        self.max_depth = 6  # depth counting top-level u as depth 1
        self.state = "BOOT"

    @classmethod
    def from_file(cls, filename: str, seed: Optional[int] = None) -> "DialogEngine":
        eng = cls(filename=filename, seed=seed)
        eng._parse_file()
        if eng.has_fatal_errors():
            eng.state = "BOOT"
        else:
            eng.state = "IDLE"
        return eng

    def has_fatal_errors(self) -> bool:
        return any(e.fatal for e in self.errors)

    def reset_to_idle(self, reason: str = "") -> None:
        if reason:
            print(f"[DIALOG] reset to IDLE: {reason}")
        self.scope_stack = []
        self.unmatched_in_scope = 0
        self.state = "IDLE"

    def interrupt_now(self) -> None:
        self.reset_to_idle(reason="global interrupt")

    def current_scope_depth(self) -> int:
        return len(self.scope_stack)

    def _set_scope_state(self) -> None:
        if self.scope_stack:
            self.state = f"IN_SCOPE({len(self.scope_stack)})"
        else:
            self.state = "IDLE"

    def _parse_file(self) -> None:
        last_by_level: Dict[int, Rule] = {}
        order = 0
        with open(self.filename, "r", encoding="utf-8") as f:
            for line_no, raw in enumerate(f, start=1):
                line = _strip_comments(raw).strip()
                if not line:
                    continue

                def_match = DEF_RE.match(line)
                if def_match:
                    name = def_match.group(1)
                    rhs = def_match.group(2).strip()
                    if not (rhs.startswith("[") and rhs.endswith("]")):
                        self.errors.append(
                            ParseError(
                                self.filename,
                                line_no,
                                "definition",
                                "definition must use [ ... ] list",
                                fatal=False,
                            )
                        )
                        continue
                    try:
                        items = parse_choice_items(rhs[1:-1])
                    except ValueError as ex:
                        self.errors.append(
                            ParseError(
                                self.filename,
                                line_no,
                                "definition",
                                str(ex),
                                fatal=False,
                            )
                        )
                        continue
                    if not items:
                        self.errors.append(
                            ParseError(
                                self.filename,
                                line_no,
                                "definition",
                                "empty definition list",
                                fatal=False,
                            )
                        )
                        continue
                    self.definitions[name] = items
                    continue

                rule_match = RULE_RE.match(line)
                if not rule_match:
                    self.errors.append(
                        ParseError(
                            self.filename,
                            line_no,
                            "syntax",
                            "line is not a valid definition or rule",
                            fatal=False,
                        )
                    )
                    continue

                level = int(rule_match.group(1) or "0")
                pattern = rule_match.group(2).strip()
                output = rule_match.group(3).strip()

                if level > 7:
                    self.errors.append(
                        ParseError(
                            self.filename,
                            line_no,
                            "nesting",
                            "rule level too deep to be usable",
                            fatal=False,
                        )
                    )
                    continue

                if pattern.count("[") != pattern.count("]"):
                    self.errors.append(
                        ParseError(
                            self.filename,
                            line_no,
                            "pattern",
                            "unbalanced [] in pattern",
                            fatal=False,
                        )
                    )
                    continue

                if output.count("[") != output.count("]"):
                    self.errors.append(
                        ParseError(
                            self.filename,
                            line_no,
                            "output",
                            "unbalanced [] in output",
                            fatal=False,
                        )
                    )
                    continue

                rule = Rule(level=level, pattern=pattern, output=output, line=line_no, order=order)
                order += 1

                if level == 0:
                    self.top_rules.append(rule)
                else:
                    parent = last_by_level.get(level - 1)
                    if parent is None:
                        self.errors.append(
                            ParseError(
                                self.filename,
                                line_no,
                                "nesting",
                                f"u{level} has no active parent u{level-1}",
                                fatal=False,
                            )
                        )
                        continue
                    parent.children.append(rule)

                last_by_level[level] = rule
                for k in list(last_by_level.keys()):
                    if k > level:
                        del last_by_level[k]

        if not self.top_rules:
            self.errors.append(
                ParseError(
                    self.filename,
                    0,
                    "fatal",
                    "no valid top-level u: rules found",
                    fatal=True,
                )
            )

    def _compile_pattern(self, pattern: str) -> Tuple[Optional[re.Pattern], List[int], Optional[str]]:
        token_regexes: List[str] = []
        capture_slots: List[int] = []
        i = 0
        slot = 0
        while i < len(pattern):
            ch = pattern[i]
            if ch.isspace():
                i += 1
                continue

            if ch == "[":
                end = _find_matching_bracket(pattern, i)
                if end < 0:
                    return None, [], "unclosed [ in pattern"
                content = pattern[i + 1 : end]
                try:
                    raw_items = parse_choice_items(content)
                except ValueError as ex:
                    return None, [], str(ex)
                if not raw_items:
                    return None, [], "empty choice in pattern"
                opts: List[str] = []
                for item in raw_items:
                    if item.startswith("~"):
                        def_name = item[1:]
                        if def_name not in self.definitions:
                            return None, [], f"undefined definition ~{def_name}"
                        for def_item in self.definitions[def_name]:
                            norm = normalize_text(def_item)
                            if norm:
                                opts.append(r"\s+".join(re.escape(t) for t in norm.split()))
                    else:
                        norm = normalize_text(item)
                        if norm:
                            opts.append(r"\s+".join(re.escape(t) for t in norm.split()))
                if not opts:
                    return None, [], "choice only had empty options after normalization"
                token_regexes.append(f"(?:{'|'.join(opts)})")
                i = end + 1
                continue

            if ch == '"':
                j = i + 1
                while j < len(pattern) and pattern[j] != '"':
                    j += 1
                if j >= len(pattern):
                    return None, [], "unclosed quote in pattern"
                literal = normalize_text(pattern[i + 1 : j])
                if literal:
                    token_regexes.append(r"\s+".join(re.escape(t) for t in literal.split()))
                i = j + 1
                continue

            if ch == "~":
                j = i + 1
                while j < len(pattern) and re.match(r"[A-Za-z0-9_]", pattern[j]):
                    j += 1
                name = pattern[i + 1 : j]
                if name not in self.definitions:
                    return None, [], f"undefined definition ~{name}"
                opts: List[str] = []
                for item in self.definitions[name]:
                    norm = normalize_text(item)
                    if norm:
                        opts.append(r"\s+".join(re.escape(t) for t in norm.split()))
                if not opts:
                    return None, [], f"definition ~{name} had no usable options"
                token_regexes.append(f"(?:{'|'.join(opts)})")
                i = j
                continue

            if ch == "_":
                slot += 1
                capture_slots.append(slot)
                token_regexes.append("(.+?)")
                i += 1
                continue

            j = i
            while j < len(pattern) and (not pattern[j].isspace()) and pattern[j] not in '[]"':
                j += 1
            literal = normalize_text(pattern[i:j])
            if literal:
                token_regexes.append(r"\s+".join(re.escape(t) for t in literal.split()))
            i = j

        if not token_regexes:
            return None, [], "empty pattern after normalization"

        rx = "^" + r"\s+".join(token_regexes) + "$"
        try:
            return re.compile(rx, re.IGNORECASE), capture_slots, None
        except re.error as ex:
            return None, [], f"regex compile error: {ex}"

    def _render_output(self, text: str) -> str:
        # Expand [ ... ] choices in output randomly.
        rendered = text
        for _ in range(20):
            match = re.search(r"\[([^\[\]]+)\]", rendered)
            if not match:
                break
            raw = match.group(1)
            try:
                items = parse_choice_items(raw)
            except ValueError:
                items = []
            replacement = self.rng.choice(items) if items else ""
            rendered = rendered[: match.start()] + replacement + rendered[match.end() :]

        # Expand ~definition in output to random option.
        def_re = re.compile(r"~([A-Za-z_][A-Za-z0-9_]*)")

        def repl_def(m: re.Match) -> str:
            name = m.group(1)
            vals = self.definitions.get(name)
            if not vals:
                return ""
            return self.rng.choice(vals)

        rendered = def_re.sub(repl_def, rendered)

        # Replace variables, unknown -> "I don't know"
        def repl_var(m: re.Match) -> str:
            name = m.group(1)
            val = self.variables.get(name)
            return val if val else "I don't know"

        rendered = VAR_RE.sub(repl_var, rendered)

        rendered = SPACE_RE.sub(" ", rendered).strip()
        return rendered

    def _extract_actions(self, text: str) -> Tuple[str, List[str]]:
        actions = ACTION_RE.findall(text)
        spoken = ACTION_RE.sub(" ", text)
        spoken = SPACE_RE.sub(" ", spoken).strip()
        return spoken, actions

    def _find_match(self, rules: List[Rule], normalized_input: str) -> Optional[Tuple[Rule, re.Match]]:
        for rule in rules:
            rx, _, err = self._compile_pattern(rule.pattern)
            if err or rx is None:
                self.errors.append(
                    ParseError(
                        self.filename,
                        rule.line,
                        "pattern",
                        f"runtime pattern error: {err}",
                        fatal=False,
                    )
                )
                continue
            m = rx.match(normalized_input)
            if m:
                return (rule, m)
        return None

    def handle_input(self, user_text: str) -> Dict[str, object]:
        if self.has_fatal_errors():
            return {
                "ok": False,
                "error": "dialog script has fatal errors",
                "state": self.state,
                "speak_text": "",
                "actions": [],
                "interrupt": False,
            }

        normalized = normalize_text(user_text)
        if not normalized:
            self._set_scope_state()
            return {
                "ok": True,
                "matched": False,
                "state": self.state,
                "speak_text": "",
                "actions": [],
                "interrupt": False,
            }

        words = set(normalized.split())
        if words & INTERRUPT_WORDS:
            self.interrupt_now()
            return {
                "ok": True,
                "matched": True,
                "state": self.state,
                "speak_text": "Stopping now.",
                "actions": [],
                "interrupt": True,
            }

        scoped_rules: List[Rule] = []
        if self.scope_stack:
            scoped_rules = list(self.scope_stack[-1].children)

        matched = self._find_match(scoped_rules, normalized)
        used_scoped = matched is not None
        if matched is None:
            matched = self._find_match(self.top_rules, normalized)
            used_scoped = False

        if matched is None:
            if self.scope_stack:
                self.unmatched_in_scope += 1
                if self.unmatched_in_scope >= 4:
                    self.reset_to_idle("4 unmatched inputs in nested scope")
            print(f"[DIALOG] no match for input='{user_text}' state={self.state}")
            self._set_scope_state()
            return {
                "ok": True,
                "matched": False,
                "state": self.state,
                "speak_text": "",
                "actions": [],
                "interrupt": False,
            }

        rule, match_obj = matched
        self.unmatched_in_scope = 0

        # Guard against activating depth beyond max_depth.
        depth_after_match = rule.level + 1
        if depth_after_match > self.max_depth:
            self.reset_to_idle(f"max depth exceeded at line {rule.line}")
            self.errors.append(
                ParseError(
                    self.filename,
                    rule.line,
                    "depth_guard",
                    f"attempted to activate depth {depth_after_match} > {self.max_depth}",
                    fatal=False,
                )
            )
            return {
                "ok": True,
                "matched": False,
                "state": self.state,
                "speak_text": "",
                "actions": [],
                "interrupt": False,
            }

        # Update scope stack.
        if used_scoped:
            while len(self.scope_stack) > rule.level:
                self.scope_stack.pop()
            self.scope_stack.append(rule)
        else:
            self.scope_stack = [rule]

        # Variable capture heuristic:
        # if pattern includes "_" and output references $name, map first capture -> first variable.
        _, capture_slots, _ = self._compile_pattern(rule.pattern)
        if capture_slots:
            captures = [g.strip() for g in match_obj.groups()]
            var_refs = VAR_RE.findall(rule.output)
            if captures and var_refs:
                self.variables[var_refs[0]] = captures[0]

        rendered = self._render_output(rule.output)
        spoken, actions = self._extract_actions(rendered)
        self._set_scope_state()

        print(f"[DIALOG] matched line={rule.line} level=u{rule.level} state={self.state}")
        return {
            "ok": True,
            "matched": True,
            "state": self.state,
            "speak_text": spoken,
            "actions": actions,
            "interrupt": False,
        }
