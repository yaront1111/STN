from ins_sim import simulate_straight_and_level
if __name__ == "__main__":
    imu_csv, truth_csv = simulate_straight_and_level()
    print("Wrote:", imu_csv, truth_csv)
