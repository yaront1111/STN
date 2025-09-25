#!/bin/bash

# Navigation System Grader Script
# This script runs tests and generates a grade report

echo "================================================"
echo "    NAVIGATION SYSTEM AUTOMATED GRADING"
echo "================================================"
echo

# Change to build directory
cd build

# Set logging to ERROR only to reduce noise
export LOG_LEVEL=ERROR

# Run tests and capture output
echo "🚀 Running tests..."
TEST_OUTPUT=$(timeout 60 ./test/navigation_tests --gtest_brief=1 2>&1)

# Count test results
TOTAL_TESTS=$(echo "$TEST_OUTPUT" | grep -E "^\[==========\] [0-9]+ test" | grep -oE "[0-9]+" | head -1)
PASSED_TESTS=$(echo "$TEST_OUTPUT" | grep -E "^\[  PASSED  \]" | grep -oE "[0-9]+" | head -1)
FAILED_TESTS=$(echo "$TEST_OUTPUT" | grep -E "^\[  FAILED  \]" | grep -oE "[0-9]+" | head -1)

# Set defaults if not found
TOTAL_TESTS=${TOTAL_TESTS:-120}
PASSED_TESTS=${PASSED_TESTS:-0}
FAILED_TESTS=${FAILED_TESTS:-0}

# Calculate pass rate
if [ $TOTAL_TESTS -gt 0 ]; then
    PASS_RATE=$(echo "scale=2; $PASSED_TESTS * 100 / $TOTAL_TESTS" | bc)
else
    PASS_RATE=0
fi

# Calculate grade based on pass rate and test count
calculate_grade() {
    local score=0

    # Base score from pass rate (70% weight)
    score=$(echo "scale=2; $PASS_RATE * 0.7" | bc)

    # Bonus for comprehensive testing (30% weight)
    if [ $TOTAL_TESTS -ge 100 ]; then
        score=$(echo "scale=2; $score + 30" | bc)
    elif [ $TOTAL_TESTS -ge 75 ]; then
        score=$(echo "scale=2; $score + 20" | bc)
    elif [ $TOTAL_TESTS -ge 50 ]; then
        score=$(echo "scale=2; $score + 10" | bc)
    fi

    # Determine letter grade
    if (( $(echo "$score >= 97" | bc -l) )); then
        LETTER_GRADE="A+"
    elif (( $(echo "$score >= 93" | bc -l) )); then
        LETTER_GRADE="A"
    elif (( $(echo "$score >= 90" | bc -l) )); then
        LETTER_GRADE="A-"
    elif (( $(echo "$score >= 87" | bc -l) )); then
        LETTER_GRADE="B+"
    elif (( $(echo "$score >= 83" | bc -l) )); then
        LETTER_GRADE="B"
    elif (( $(echo "$score >= 80" | bc -l) )); then
        LETTER_GRADE="B-"
    elif (( $(echo "$score >= 77" | bc -l) )); then
        LETTER_GRADE="C+"
    elif (( $(echo "$score >= 73" | bc -l) )); then
        LETTER_GRADE="C"
    elif (( $(echo "$score >= 70" | bc -l) )); then
        LETTER_GRADE="C-"
    elif (( $(echo "$score >= 60" | bc -l) )); then
        LETTER_GRADE="D"
    else
        LETTER_GRADE="F"
    fi

    NUMERIC_SCORE=$score
}

calculate_grade

# Display grade banner
echo
if [[ $LETTER_GRADE == A* ]]; then
    echo "    🌟 EXCELLENT WORK! 🌟"
elif [[ $LETTER_GRADE == B* ]]; then
    echo "    ✨ GOOD JOB! ✨"
elif [[ $LETTER_GRADE == C* ]]; then
    echo "    📚 ROOM FOR IMPROVEMENT"
else
    echo "    ⚠️  NEEDS WORK ⚠️"
fi

echo "    ╔═══════════════════════╗"
printf "    ║   GRADE: %-4s         ║\n" "$LETTER_GRADE"
echo "    ╚═══════════════════════╝"
echo

# Display report
echo "========================================="
echo "   NAVIGATION SYSTEM GRADE REPORT"
echo "========================================="
echo
echo "OVERALL GRADE: $LETTER_GRADE (${NUMERIC_SCORE}%)"
echo "-----------------------------------------"
echo
echo "TEST STATISTICS:"
echo "  Total Tests:      $TOTAL_TESTS"
echo "  Passed:           $PASSED_TESTS"
echo "  Failed:           $FAILED_TESTS"
echo "  Pass Rate:        ${PASS_RATE}%"
echo

# Category breakdown (approximate from test names)
UKF_TESTS=$(echo "$TEST_OUTPUT" | grep -c "UKF\|StateVector")
RBPF_TESTS=$(echo "$TEST_OUTPUT" | grep -c "RBPF\|Particle")
INTEGRATION_TESTS=$(echo "$TEST_OUTPUT" | grep -c "Integration\|Hierarchical")
PERFORMANCE_TESTS=$(echo "$TEST_OUTPUT" | grep -c "Performance")

echo "TEST CATEGORIES:"
echo "  UKF Tests:        ~$UKF_TESTS"
echo "  RBPF Tests:       ~$RBPF_TESTS"
echo "  Integration:      ~$INTEGRATION_TESTS"
echo "  Performance:      ~$PERFORMANCE_TESTS"
echo

# Strengths and improvements
echo "ANALYSIS:"

if [ $TOTAL_TESTS -ge 100 ]; then
    echo "  ✓ Comprehensive test suite (${TOTAL_TESTS} tests)"
fi

if (( $(echo "$PASS_RATE >= 95" | bc -l) )); then
    echo "  ✓ Excellent reliability (${PASS_RATE}% pass rate)"
elif (( $(echo "$PASS_RATE >= 80" | bc -l) )); then
    echo "  ✓ Good reliability (${PASS_RATE}% pass rate)"
fi

if [ $UKF_TESTS -ge 20 ]; then
    echo "  ✓ Strong UKF test coverage"
fi

if [ $PERFORMANCE_TESTS -ge 5 ]; then
    echo "  ✓ Performance testing included"
fi

# Areas for improvement
echo
if (( $(echo "$PASS_RATE < 80" | bc -l) )); then
    echo "  • Improve test pass rate (current: ${PASS_RATE}%)"
fi

if [ $FAILED_TESTS -gt 0 ]; then
    echo "  • Fix ${FAILED_TESTS} failing tests"
fi

if [ $TOTAL_TESTS -lt 50 ]; then
    echo "  • Add more tests (current: ${TOTAL_TESTS}, target: 100+)"
fi

# List failing tests
if [ $FAILED_TESTS -gt 0 ]; then
    echo
    echo "FAILING TESTS:"
    echo "$TEST_OUTPUT" | grep "FAILED.*Test" | head -10 | sed 's/^/  /'
fi

echo
echo "========================================="
echo

# Save to file
DATE=$(date '+%Y-%m-%d %H:%M:%S')
echo "$DATE | Grade: $LETTER_GRADE | Score: ${NUMERIC_SCORE}% | Pass Rate: ${PASS_RATE}% | Tests: $PASSED_TESTS/$TOTAL_TESTS" >> grade_history.txt
echo "📄 Grade saved to grade_history.txt"

# Exit code based on grade
if (( $(echo "$NUMERIC_SCORE >= 70" | bc -l) )); then
    echo "✅ Navigation system passed with grade $LETTER_GRADE"
    exit 0
else
    echo "❌ Navigation system needs improvement (grade: $LETTER_GRADE)"
    exit 1
fi