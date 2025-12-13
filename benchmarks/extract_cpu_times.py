import sys
import re

def main():
    if len(sys.argv) != 2:
        print("Usage: python cpu_stats.py <file_path>")
        sys.exit(1)

    file_path = sys.argv[1]
    cpu_times = []
    pattern = re.compile(r"CPU time \(s\):\s+([0-9]*\.?[0-9]+)")

    lines = []
    try:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except UnicodeDecodeError:
            with open(file_path, 'r', encoding='utf-16') as f:
                lines = f.readlines()

        for line in lines:
            match = pattern.search(line)
            if match:
                cpu_times.append(float(match.group(1)))

    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)

    if not cpu_times:
        print("No matching CPU time lines found.")
        return

    avg_time = sum(cpu_times) / len(cpu_times)
    max_time = max(cpu_times)

    print(f"{'Metric':<10} {'Value (s)'}")
    print("-" * 25)
    print(f"{'Count':<10} {len(cpu_times)}")
    print(f"{'Average':<10} {avg_time:.6f}")
    print(f"{'Max':<10} {max_time:.6f}")

if __name__ == "__main__":
    main()