import re
import difflib
import subprocess

VALID_LABELS = [
    "banana", "marker", "rubiks_cube",
    "green_cube", "red_cube", "yellow_cube",
    "scissor", "clamp", "allen_key", "measuring_tape",
    "knife", "chisel"
]

PICK_SYNONYMS = ["pick", "take", "grab", "collect", "lift", "get"]

LABEL_SYNONYMS = {
    "red_cube": ["red block", "red box"],
    "green_cube": ["green block", "green box"],
    "yellow_cube": ["yellow block", "yellow box"],
    "rubiks_cube": ["rubik's cube", "rubix cube"],
    "measuring_tape": ["tape", "measuring tape roll", "tape measure"],
    "clamp": ["clamp tool", "clamp device"],
    # add more as needed
}

_last_suggestion = None      # for typo confirmation
_pending_pick = None         # stores label until box chosen


def find_best_label(user_text: str) -> str | None:
    words = user_text.lower().split()
    best_match = None
    best_ratio = 0.0

    for label in VALID_LABELS:
        candidates = [label] + label.split("_")
        if label in LABEL_SYNONYMS:
            candidates += LABEL_SYNONYMS[label]

        for word in words:
            for cand in candidates:
                ratio = difflib.SequenceMatcher(None, word, cand.lower()).ratio()
                if ratio > best_ratio:
                    best_match = label
                    best_ratio = ratio

    return best_match if best_ratio > 0.6 else None


def llm_chat_or_pick(user_text: str, visible_objects: set[str] | None = None) -> dict:
    """
    Main LLM + pick handling function.
    visible_objects: current set of detected objects from YOLO
    """
    global _last_suggestion, _pending_pick
    text = user_text.lower().strip()

    # --- Handle confirmation (typos) ---
    if _last_suggestion:
        if text in ["yes", "y", "yeah", "ok", "sure"]:
            label = _last_suggestion
            _last_suggestion = None

            # Only proceed if object is visible
            if visible_objects and label not in visible_objects:
                return {"type": "chat", "reply": f"I cannot see {label.replace('_',' ')} in the picking tray 😕"}

            _pending_pick = label
            return {"type": "ask_box", "label": label,
                    "reply": f"Okay, {label.replace('_',' ')} selected ✅. In which box do you want to drop it (1 or 2)?"}

        elif text in ["no", "n", "nope"]:
            _last_suggestion = None
            return {"type": "chat", "reply": "Alright, cancelled ❌. What should I grab instead?"}

    # --- Handle box choice ---
    if _pending_pick:
        if "1" in text:
            label = _pending_pick
            _pending_pick = None
            return {"type": "pick", "label": label, "box": "1",
                    "reply": f"Picking the {label.replace('_',' ')} ⏳ in progress..."}
        elif "2" in text:
            label = _pending_pick
            _pending_pick = None
            return {"type": "pick", "label": label, "box": "2",
                    "reply": f"Picking the {label.replace('_',' ')} ⏳ in progress..."}
        else:
            return {"type": "chat", "reply": "Please specify box 1 or box 2 📦"}

    # --- Detect pick intent ---
    if any(re.search(rf"\b{syn}\b", text) for syn in PICK_SYNONYMS):
        for label in VALID_LABELS:
            pattern = label.replace("_", "[ _]")
            if re.search(rf"\b{pattern}\b", text):
                # Only ask for box if visible
                if visible_objects and label not in visible_objects:
                    return {"type": "chat", "reply": f"I cannot see {label.replace('_',' ')} in the picking tray 😕"}

                _pending_pick = label
                return {"type": "ask_box", "label": label,
                        "reply": f"Okay, {label.replace('_',' ')} selected ✅. In which box do you want to drop it (1 or 2)?"}

        best_match = find_best_label(text)
        if best_match:
            _last_suggestion = best_match
            return {"type": "chat", "reply": f"Did you mean **{best_match.replace('_',' ')}**? (yes/no)"}

        return {"type": "chat", "reply": "I didn’t recognize that item 🤔. Can you try again?"}

    # --- Fallback casual chat ---
    prompt = f"""
You are a helpful robot assistant. Only reply to casual chat.
User message: "{user_text}"
Respond only with text (no JSON needed).
"""
    try:
        result = subprocess.run(["ollama", "run", "phi3", prompt], capture_output=True, text=True)
        reply = result.stdout.strip() or "Hello! How can I help you?"
    except Exception:
        reply = "Hello! How can I help you?"
    return {"type": "chat", "reply": reply}
