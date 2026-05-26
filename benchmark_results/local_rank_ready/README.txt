Local rank ready package
NMPC proofs use time_aligned scoring: target columns are shifted by estimated phase_delay_seconds before scoring.
PID/RL proofs use raw scoring and still include signed phase_delay_seconds.
Proof verification checks result.csv hash/size/row_count plus the signed manifest.
