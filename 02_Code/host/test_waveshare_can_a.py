import struct
import unittest

from waveshare_can_a import (
    CAN_ID_ACK,
    CAN_ID_POINT_A,
    CAN_ID_POINT_B,
    CMD_CLEAR_DATA,
    CanFrame,
    VariableFrameParser,
    apply_calibration_preset,
    apply_locked_final_matrix_experiment,
    apply_locked_15v_experiment,
    build_adapter_config_packet,
    build_parser,
    config_a_frame,
    config_b_frame,
    command_frame,
    decode_application_frame,
    encode_variable_frame,
    pid_config_frame,
    retrieve_point,
)


class WaveshareCodecTests(unittest.TestCase):
    def test_official_standard_frame_vector(self) -> None:
        frame = CanFrame(0x123, bytes.fromhex("11 22 33 44 55 66 77 88"))
        encoded = bytes.fromhex("AA C8 23 01 11 22 33 44 55 66 77 88 55")
        self.assertEqual(encode_variable_frame(frame), encoded)
        parser = VariableFrameParser()
        self.assertEqual(parser.feed(encoded[:4]), [])
        self.assertEqual(parser.feed(encoded[4:]), [frame])

    def test_parser_recovers_after_noise(self) -> None:
        frame = CanFrame(0x103, bytes.fromhex("11 22"))
        encoded = bytes.fromhex("AA C2 03 01 11 22 55")
        parser = VariableFrameParser()
        self.assertEqual(parser.feed(b"\x00\x99" + encoded), [frame])

    def test_adapter_config_checksum(self) -> None:
        packet = build_adapter_config_packet(500_000)
        self.assertEqual(len(packet), 20)
        self.assertEqual(packet[:5], bytes.fromhex("AA 55 12 03 01"))
        self.assertEqual(packet[-1], sum(packet[2:19]) & 0xFF)

    def test_known_sweep_payloads(self) -> None:
        self.assertEqual(
            config_a_frame(0x0B, 1800, 2000, 5).data,
            bytes.fromhex("0B 00 08 07 D0 07 05 00"),
        )
        self.assertEqual(
            config_b_frame(0x0B, 3, 100, 40, 45, 700).data,
            bytes.fromhex("0B 03 64 00 28 2D BC 02"),
        )

    def test_clear_previous_sweep_payload(self) -> None:
        self.assertEqual(
            command_frame(CMD_CLEAR_DATA, 0x55).data,
            bytes.fromhex("07 55 00 00 00 00 00 00"),
        )

    def test_fetch_parser(self) -> None:
        args = build_parser().parse_args(["fetch", "--count", "41", "--plot"])
        self.assertEqual(args.command, "fetch")
        self.assertEqual(args.count, 41)
        self.assertTrue(args.plot)

    def test_point_retrieval_retries_missing_frame(self) -> None:
        class DropFirstPointBAdapter:
            def __init__(self) -> None:
                self.send_count = 0
                self.frames = []

            def send(self, frame: CanFrame) -> None:
                self.send_count += 1
                token = frame.data[1]
                ack = CanFrame(
                    CAN_ID_ACK,
                    bytes([0x05, token, 0, 4, 0, 0, 1, 0]),
                )
                point_a = CanFrame(
                    CAN_ID_POINT_A,
                    struct.pack("<HHHh", 35, 1975, 0, 220),
                )
                point_b = CanFrame(
                    CAN_ID_POINT_B,
                    struct.pack("<HHhH", 35, 9980, 377, 1),
                )
                self.frames = [ack, point_a]
                if self.send_count > 1:
                    self.frames.append(point_b)

            def read_frames(self, timeout_s: float):
                del timeout_s
                frames, self.frames = self.frames, []
                return frames

        adapter = DropFirstPointBAdapter()
        point = retrieve_point(
            adapter,
            35,
            max_attempts=2,
            response_timeout_s=0.001,
            retry_delay_s=0.0,
        )
        self.assertEqual(adapter.send_count, 2)
        self.assertEqual(point["index"], 35)
        self.assertAlmostEqual(point["ina_current_ma"], 37.7)

    def test_ack_decode(self) -> None:
        data = struct.pack("<BBBBBBBB", 1, 7, 0, 0, 0, 1, 1, 0)
        decoded = decode_application_frame(CanFrame(CAN_ID_ACK, data))
        self.assertEqual(decoded["kind"], "ack")
        self.assertEqual(decoded["token"], 7)
        self.assertEqual(decoded["version"], 1)

    def test_pid_config_payload(self) -> None:
        self.assertEqual(
            pid_config_frame(1, 0, 500, 2010, 300).data,
            bytes.fromhex("01 00 F4 01 DA 07 2C 01"),
        )

    def test_final_pid_start_parser(self) -> None:
        args = build_parser().parse_args(["pid-start"])
        self.assertEqual(args.profile, 2)
        self.assertEqual(args.target_ma, 100.0)
        self.assertEqual(args.base_mv, 2050)
        self.assertEqual(args.ff_mv_per_c, 4.0)

    def test_live_plot_parser(self) -> None:
        args = build_parser().parse_args(
            ["live-plot", "--seconds", "30", "--output", "live.csv"]
        )
        self.assertEqual(args.seconds, 30.0)
        self.assertEqual(args.output, "live.csv")

    def test_pid_demo_parser(self) -> None:
        args = build_parser().parse_args(["pid-demo", "--seconds", "90"])
        self.assertEqual(args.target_ma, 100.0)
        self.assertEqual(args.base_mv, 2050)
        self.assertEqual(args.seconds, 90.0)

    def test_locked_15v_experiment_parser(self) -> None:
        args = build_parser().parse_args(
            ["experiment-demo", "pid-no-ff", "--seconds", "720"]
        )
        self.assertEqual(args.mode, "pid-no-ff")
        self.assertEqual(args.seconds, 720.0)
        self.assertEqual(apply_locked_15v_experiment(args), "pid-no-ff")
        self.assertEqual(args.profile, 0)
        self.assertEqual(args.target_ma, 100.0)
        self.assertEqual(args.base_mv, 2015)
        self.assertEqual(args.ff_mv_per_c, 0.0)

    def test_locked_15v_pid_ff_profile(self) -> None:
        args = build_parser().parse_args(["experiment-start", "pid-ff"])
        apply_locked_15v_experiment(args)
        self.assertEqual(args.profile, 2)
        self.assertEqual(args.target_ma, 100.0)
        self.assertEqual(args.base_mv, 2015)
        self.assertEqual(args.ff_mv_per_c, 4.0)

    def test_locked_10v_final_matrix_profile_and_filename(self) -> None:
        args = build_parser().parse_args(
            ["matrix-demo", "10", "pid-ff", "--repeat", "2"]
        )
        self.assertEqual(
            apply_locked_final_matrix_experiment(args), "10V-pid-ff"
        )
        self.assertEqual(args.profile, 2)
        self.assertEqual(args.target_ma, 50.0)
        self.assertEqual(args.base_mv, 1934)
        self.assertEqual(args.ff_mv_per_c, 3.0)
        self.assertEqual(args.seconds, 600.0)
        self.assertTrue(args.output.endswith("10V_50mA_03_PID_WITH_NTC_FF_R2.csv"))

    def test_locked_15v_final_matrix_open_loop(self) -> None:
        args = build_parser().parse_args(
            ["matrix-demo", "15", "open-loop", "--repeat", "3"]
        )
        self.assertEqual(
            apply_locked_final_matrix_experiment(args), "15V-open-loop"
        )
        self.assertEqual(args.profile, 3)
        self.assertEqual(args.target_ma, 100.0)
        self.assertEqual(args.base_mv, 2062)
        self.assertEqual(args.ff_mv_per_c, 0.0)
        self.assertTrue(args.output.endswith("15V_100mA_04_OPEN_LOOP_R3.csv"))

    def test_final_matrix_temperature_limiter_label(self) -> None:
        args = build_parser().parse_args(
            ["matrix-demo", "15", "temp-limiter", "--repeat", "1"]
        )
        self.assertEqual(
            apply_locked_final_matrix_experiment(args),
            "15V-temp-limiter",
        )

    def test_10v_calibration_sweep_preset(self) -> None:
        args = build_parser().parse_args(["cal-sweep", "10", "--repeat", "2"])
        apply_calibration_preset(args, pulse=False)
        self.assertEqual(args.start_mv, 1800)
        self.assertEqual(args.stop_mv, 2050)
        self.assertEqual(args.current_stop_ma, 70.0)
        self.assertTrue(args.plot)
        self.assertTrue(args.output.endswith("10V_bracket_R2.csv"))

    def test_15v_calibration_pulse_preset(self) -> None:
        args = build_parser().parse_args(
            ["cal-pulse", "15", "--dac-mv", "2015", "--repeat", "3"]
        )
        apply_calibration_preset(args, pulse=True)
        self.assertEqual(args.start_mv, 2015)
        self.assertEqual(args.stop_mv, 2015)
        self.assertEqual(args.samples, 7)
        self.assertEqual(args.current_stop_ma, 120.0)
        self.assertFalse(args.plot)
        self.assertTrue(args.output.endswith("15V_D2015_R3.csv"))


if __name__ == "__main__":
    unittest.main()
