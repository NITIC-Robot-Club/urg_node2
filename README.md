# urg_node2_nl (No Lifecycle) - No Lifecycle management ROS2 driver for Hokuyo URG

## Sensor timestamp synchronization

When `synchronize_time` is enabled, the driver periodically queries the URG
sensor clock with SCIP `TM` commands, extends its 24-bit millisecond counter,
and estimates the sensor-to-system clock offset and drift from low-latency
responses. Scan timestamps are mapped from the sensor timestamp and refined
against the observed scan cadence; callback arrival time is only used as a
fallback when synchronization cannot be established. The approach follows the
clock and scan-origin estimation principles in
[`seqsense/urg_stamped`](https://github.com/seqsense/urg_stamped).

`clock_sync_interval_sec` controls the re-synchronization interval (default
30 s). URG timestamp-mode synchronization temporarily stops scanning; UUST2
firmware may take longer because its TM responses are scheduled on a 5 ms
timer. Verify sensor model/firmware and the resulting stamp residuals on the
target robot before relying on this experimental path.
