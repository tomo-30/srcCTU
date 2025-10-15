# include/plan_utils.py
import os, json
from typing import List, Tuple

def load_plan_sequence(base_dir: str, plan_filename: str) -> List[Tuple[str, tuple]]:
    path = os.path.join(base_dir, "info", plan_filename)
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    plan = data.get("plan", [])
    plan_sorted = sorted(plan, key=lambda d: d.get("id", 0))
    seq = []
    for item in plan_sorted:
        _id = item.get("id")
        A = item.get("A"); B = item.get("B")
        if not (A and B) or len(A) < 6 or len(B) < 6:
            raise ValueError(f"id={_id} のA/Bが不正（[x,y,z,roll,pitch,yaw]の6要素）")
        Ax,Ay,Az,Ar,Ap,Ayaw = map(float, A)
        Bx,By,Bz,Br,Bp,Byaw = map(float, B)
        seq.append((f"A{_id}", (Ax,Ay,Az,Ar,Ap,Ayaw)))
        seq.append((f"B{_id}", (Bx,By,Bz,Br,Bp,Byaw)))
        seq.append((f"A{_id}", (Ax,Ay,Az,Ar,Ap,Ayaw)))
    return seq
