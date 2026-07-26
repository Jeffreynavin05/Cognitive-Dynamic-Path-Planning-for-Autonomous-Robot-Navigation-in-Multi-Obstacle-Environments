# Forecast for a single tracked object. model_name identifies which predictor
# produced it (e.g. "constant_velocity", "kalman_filter", "lstm_v1") so the
# predictor can be swapped without any change to consumers of this message.
uint32 track_id
string model_name
TrajectoryPoint[] points
