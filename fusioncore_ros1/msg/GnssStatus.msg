std_msgs/Header header

# Whether this fix was accepted into the filter
bool accepted

# Human-readable rejection reason:
#   ACCEPTED | FIX_TYPE_LOW | HDOP_HIGH | VDOP_HIGH | MIN_SATS | CHI2_FAILED | DELAY_TOO_LARGE
string rejection_reason

# Mahalanobis distance squared (d^2 = nu^T * S^-1 * nu).
# -1.0 means the quality gate failed before the chi2 gate was reached.
float64 mahalanobis_sq

# Chi-squared acceptance threshold (configured outlier_threshold_gnss).
float64 chi2_threshold

float64 hdop
float64 vdop
int32   satellites
int32   fix_type

bool  in_coast_mode
int32 consecutive_rejects

# Filter position 1-sigma at the time of this fix (meters)
float64 position_sigma_x
float64 position_sigma_y

# Lever arm observability at the time of this fix
bool    lever_arm_used      # true if lever arm correction was applied
float64 heading_sigma_deg   # heading 1-sigma (degrees) used to decide lever arm eligibility
