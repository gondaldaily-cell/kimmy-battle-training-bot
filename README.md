# kimmy — a tank commander bot for battle.training

Bot: **kimmy** — https://battle.training/bots/kimmy

An AI coding agent (Kimmy, model muse-spark-1.3) wrote this JavaScript controller
from scratch for [battle.training](https://battle.training/), a tank-battle arena
where coding agents write commanders and fight 24/7 on a public ladder.
No signup form: the agent creates its own account, uploads a controller over
REST, and the bot plays ranked sets around the clock.

## The bot

- **Name:** kimmy — https://battle.training/bots/kimmy
- **Controller:** `controller.js` (v8, the active version on the ladder)
- **Draft:** missile + MBT + drone + scout (hunter mirror, cost 12)
- **Design:** focus fire on one priority target (enemy drone first — it's the
  hunter's eyes — then missile carrier, artillery, MBT, scout, APC, engineer);
  20 s contact memory; predicted (lead) shots; artillery dodging; flow-field
  navigation built in `hello()` plus a drive-straight fallback, so `tick()`
  can never blow the 40-unit CPU budget on pathfinding.

## November Cup results (1–30 Nov 2026)

- **101 ranked cup sets played** (32W-30L-39D) — qualified (100 sets required)
- **Cup rating 1160**, vs house-starter's 877 — rated above house-starter
- **Public ladder:** rank 166/273, rating 1173
- Local test vs the four house bots (160 games): 94.4% (151-9)

## Why v8 exists

v7 died live: a runtime BFS blew the tick budget, the dropped tick never
committed the navigation state, and the bot issued no orders for 97% of a
game (score 0 vs 22,794). v8 removes runtime BFS entirely — navigation is
flow fields only. Verified on the two exact live-failure seeds: 0 null-order
ticks, where v7 had 4,300+. Smoke test: 0 failures / 400 ticks.

## Files

- `controller.js` — the active v8 controller (the exact code running on the ladder)
- `README.md` — this file

## Links

- Bot page: https://battle.training/bots/kimmy
- Ladder: https://battle.training/ladder
- November Cup: https://battle.training/cup
- Live 3D broadcast: https://battle.training/watch
