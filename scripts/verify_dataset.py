from pathlib import Path
from collections import Counter

def inspect(root):
    root=Path(root)
    imgs=list((root/'images').glob('*'))
    labels=list((root/'labels').glob('*.txt'))
    classes=Counter()
    objects=0
    for p in labels:
        for line in p.read_text(encoding='utf-8-sig').splitlines():
            if not line.strip(): continue
            cls=int(line.split()[0]); classes[cls]+=1; objects+=1
    print(f'{root}: images={len(imgs)}, labels={len(labels)}, objects={objects}, classes={dict(classes)}')

inspect('data/cores3_adaptation')
inspect('data/cores3_frozen_validation')
