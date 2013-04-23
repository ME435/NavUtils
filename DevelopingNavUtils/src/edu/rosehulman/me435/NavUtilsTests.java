package edu.rosehulman.me435;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.Before;
import org.junit.Test;

public class NavUtilsTests {

	private static final double EPSILON = NavUtils.BISECTION_ERROR_TOLERANCE_FT;

	// #1. Pointing straight back with a radius of 10.
	double ROBOT_X1 = 10, ROBOT_Y1 = 10, ROBOT_HEADING_1 = 180, TARGET_X1 = 0,
			TARGET_Y1 = 0;
	double CENTER_X1 = 10, CENTER_Y1 = 0, RADIUS_1 = -10,
			ARC_LENGTH_1 = 15.70796;

	// #2. Pointing perfectly at the target (same robot XY as #1).
	double ROBOT_X2 = 10, ROBOT_Y2 = 10, ROBOT_HEADING_2 = -135, TARGET_X2 = 0,
			TARGET_Y2 = 0;
	double CENTER_X2 = 10 - 707.1067811, CENTER_Y2 = 10 + 707.1067811,
			RADIUS_2 = NavUtils.MAX_RADIUS_FT, ARC_LENGTH_2 = 14.142135;

	// #3. Pointing back left with a radius of 10 (same circle center as #1).
	double ROBOT_X3 = 17.071067, ROBOT_Y3 = 7.0710678, ROBOT_HEADING_3 = 135,
			TARGET_X3 = 0, TARGET_Y3 = 0;
	double CENTER_X3 = 10, CENTER_Y3 = 0, RADIUS_3 = -10,
			ARC_LENGTH_3 = 23.5619449; // 3/8 of the circle perimeter.

	@Before
	public void setUp() throws Exception {
	}

	@Test
	public void testCalculateArc() {
		double[] arcResult = new double[2];
		assertTrue(NavUtils.calculateArc(ROBOT_X1, ROBOT_Y1, ROBOT_HEADING_1,
				TARGET_X1, TARGET_Y1, arcResult));
		assertEquals(RADIUS_1, arcResult[0], EPSILON);
		assertEquals(ARC_LENGTH_1, arcResult[1], EPSILON);
		assertTrue(NavUtils.calculateArc(ROBOT_X2, ROBOT_Y2, ROBOT_HEADING_2,
				TARGET_X2, TARGET_Y2, arcResult));
		assertEquals(RADIUS_2, arcResult[0], EPSILON);
		assertEquals(ARC_LENGTH_2, arcResult[1], EPSILON);
		assertTrue(NavUtils.calculateArc(ROBOT_X3, ROBOT_Y3, ROBOT_HEADING_3,
				TARGET_X3, TARGET_Y3, arcResult));
		assertEquals(RADIUS_3, arcResult[0], EPSILON);
		assertEquals(ARC_LENGTH_3, arcResult[1], EPSILON);
	}

	@Test
	public void testGetTargetHeading() {
		assertEquals(-135, NavUtils.getTargetHeading(ROBOT_X1, ROBOT_Y1,
				TARGET_X1, TARGET_Y1), EPSILON);
		assertEquals(-135, NavUtils.getTargetHeading(ROBOT_X2,
				ROBOT_Y2, TARGET_X2, TARGET_Y2), EPSILON);
		assertEquals(-(180 - 22.5000), NavUtils.getTargetHeading(ROBOT_X3, ROBOT_Y3,
				TARGET_X3, TARGET_Y3), EPSILON);
	}

	@Test
	public void testTargetIsOnLeft() {
		assertTrue(NavUtils.targetIsOnLeft(ROBOT_X1, ROBOT_Y1, ROBOT_HEADING_1,
				TARGET_X1, TARGET_Y1));
		assertFalse(NavUtils.targetIsOnLeft(ROBOT_X2, ROBOT_Y2,
				ROBOT_HEADING_2, TARGET_X2, TARGET_Y2));
		assertTrue(NavUtils.targetIsOnLeft(ROBOT_X3, ROBOT_Y3, ROBOT_HEADING_3,
				TARGET_X3, TARGET_Y3));
	}

	@Test
	public void testTurnHeadingDelta() {
		assertEquals(90, NavUtils.getLeftTurnHeadingDelta(135, -135), EPSILON);
		assertEquals(270, NavUtils.getRightTurnHeadingDelta(135, -135), EPSILON);
		assertEquals(0, NavUtils.getLeftTurnHeadingDelta(45, 45), EPSILON);
		assertEquals(0, NavUtils.getRightTurnHeadingDelta(45, 45), EPSILON);
		assertEquals(15, NavUtils.getLeftTurnHeadingDelta(30, 45), EPSILON);
		assertEquals(345, NavUtils.getRightTurnHeadingDelta(30, 45), EPSILON);
	}

	@Test
	public void testGetDistance() {
		assertEquals(14.142135, NavUtils.getDistance(0, 0, 10, 10), EPSILON);
		assertEquals(10, NavUtils.getDistance(0, 0, 10, 0), EPSILON);
		assertEquals(14.142135, NavUtils.getDistance(0, 10, 10, 0), EPSILON);

	}

	@Test
	public void testNormalizeAngle() {
		assertEquals(30.0, NavUtils.normalizeAngle(390.0), EPSILON);
		assertEquals(-30.0, NavUtils.normalizeAngle(330.0), EPSILON);
	}

	@Test
	public void testBisectionForRadius() {
		assertEquals(RADIUS_1, NavUtils.bisectionForRadius(
				-NavUtils.MIN_RADIUS_FT, -NavUtils.MAX_RADIUS_FT,
				NavUtils.BISECTION_ERROR_TOLERANCE_FT, ROBOT_X1, ROBOT_Y1,
				ROBOT_HEADING_1, TARGET_X1, TARGET_Y1), EPSILON);
		// Note: #2 should use bisection.
		assertEquals(RADIUS_3, NavUtils.bisectionForRadius(
				-NavUtils.MIN_RADIUS_FT, -NavUtils.MAX_RADIUS_FT,
				NavUtils.BISECTION_ERROR_TOLERANCE_FT, ROBOT_X3, ROBOT_Y3,
				ROBOT_HEADING_3, TARGET_X3, TARGET_Y3), EPSILON);

	}

	@Test
	public void testCircleContainsTarget() {
		
		// Note R1 is negative so subtraction makes it bigger.
		assertTrue(NavUtils.circleContainsTarget(RADIUS_1 - 0.1, ROBOT_X1,
				ROBOT_Y1, ROBOT_HEADING_1, TARGET_X1, TARGET_Y1));
		assertFalse(NavUtils.circleContainsTarget(RADIUS_1 + 0.1, ROBOT_X1,
				ROBOT_Y1, ROBOT_HEADING_1, TARGET_X1, TARGET_Y1));
		assertFalse(NavUtils.circleContainsTarget(RADIUS_2, ROBOT_X2,
				ROBOT_Y2, ROBOT_HEADING_2, TARGET_X2, TARGET_Y2));
		// Note R3 is negative so subtraction makes it bigger.
		assertTrue(NavUtils.circleContainsTarget(RADIUS_3 - 0.1, ROBOT_X3,
				ROBOT_Y3, ROBOT_HEADING_3, TARGET_X3, TARGET_Y3));
		assertFalse(NavUtils.circleContainsTarget(RADIUS_3 + 0.1, ROBOT_X3,
				ROBOT_Y3, ROBOT_HEADING_3, TARGET_X3, TARGET_Y3));
	}

	@Test
	public void testCalculateCircleCenter() {
		double[] centerPoint = new double[2];
		NavUtils.calculateCircleCenter(RADIUS_1, ROBOT_X1, ROBOT_Y1,
				ROBOT_HEADING_1, centerPoint);
		assertEquals(CENTER_X1, centerPoint[0], EPSILON);
		assertEquals(CENTER_Y1, centerPoint[1], EPSILON);
		NavUtils.calculateCircleCenter(RADIUS_2, ROBOT_X2, ROBOT_Y2,
				ROBOT_HEADING_2, centerPoint);
		assertEquals(CENTER_X2, centerPoint[0], EPSILON);
		assertEquals(CENTER_Y2, centerPoint[1], EPSILON);
		NavUtils.calculateCircleCenter(RADIUS_3, ROBOT_X3, ROBOT_Y3,
				ROBOT_HEADING_3, centerPoint);
		assertEquals(CENTER_X3, centerPoint[0], EPSILON);
		assertEquals(CENTER_Y3, centerPoint[1], EPSILON);
	}

	@Test
	public void testGetArcLength() {
		assertEquals(ARC_LENGTH_1, NavUtils.getArcLength(RADIUS_1, ROBOT_X1,
				ROBOT_Y1, ROBOT_HEADING_1, TARGET_X1, TARGET_Y1), EPSILON);
		assertEquals(ARC_LENGTH_2, NavUtils.getArcLength(RADIUS_2, ROBOT_X2,
				ROBOT_Y2, ROBOT_HEADING_2, TARGET_X2, TARGET_Y2), EPSILON);
		assertEquals(ARC_LENGTH_3, NavUtils.getArcLength(RADIUS_3, ROBOT_X3,
				ROBOT_Y3, ROBOT_HEADING_3, TARGET_X3, TARGET_Y3), EPSILON);
	}

}
