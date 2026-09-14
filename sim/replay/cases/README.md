# Registered real-bike regressions

Each case lives in `sim/replay/cases/<case>/` with a canonical `input.csv` and `manifest.json`.

## What a case is checked for

`tools/run_replay_regression.py` reports three things separately, because they are three different
claims and printing one `CASE PASS` for all of them promised more than had been checked:

| Verdict | Question |
|---|---|
| `REPLAY_EXECUTED` | did the production chain consume the recorded sensor history and produce a trace? |
| `BEHAVIOR_ACCEPTED` | does that trace satisfy the criteria in this case's `manifest.json`? |
| `OUTPUT_PINNED` | do the bytes match a hash a human accepted after validating that ride? |

A case whose manifest states no criteria reports `BEHAVIOR_NOT_ASSESSED`. That is deliberate: a
newly captured ride can be registered before anyone knows what to require of it, and until someone
does, the gate says so out loud instead of counting it as a pass.

## Stating criteria

Add a `behavior` object to the manifest. Every key is optional; state only what the fragment can
actually settle.

```json
"behavior": {
  "produces_assist":  true,
  "responds_to_load": "rising",
  "pause_releases":   0.95,
  "restart_recovers": true,
  "max_attenuation":  0.60,
  "note":             "why these and not others"
}
```

| Key | Asks |
|---|---|
| `produces_assist` | any current at all came out |
| `responds_to_load` | `"rising"` / `"falling"` — the assist moved the same way the pedal force did |
| `pause_releases` | the fraction of the not-pedalling samples that must be at zero current |
| `restart_recovers` | current returns after the pause ends |
| `max_attenuation` | the current's peak-to-peak swing, relative to the pedal's, stays under this |

`max_attenuation` **refuses** a fragment whose request sits on its ceiling for more than 10 % of
the samples: a clipped signal is smooth because it is clipped, and reporting that as attenuation
would pass for the wrong reason. Give such fragments `responds_to_load` instead. The limit is the
same `0.60` used by `tools/analyze_assist_ripple.py` — one standard, not two.

Criteria are properties, not frozen numbers. They must survive tuning and must fail if the
architecture regresses. `tests/test_replay_behavior.py` proves each one can reject as well as
accept; a criterion nobody has watched fail is indistinguishable from one that always passes.

## Pinning an output

A case without `accepted_output_sha256` is replayed for invariants, determinism and behaviour, but
does not freeze the old output. After a fix is reviewed, an accepted output hash may be stored so
future firmware changes cannot silently alter that ride.

Do **not** accept a baseline from a known-bug firmware merely to make the gate green.
