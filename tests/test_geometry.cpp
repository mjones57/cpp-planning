#include <gtest/gtest.h>
#include "planning/grid_map.hpp"
#include "planning/geometry.hpp"

using namespace planning;

TEST(Geometry, CellOfFloorsCoordinates)
{
    // Ensure cell_of maps Point2->Cell properly for OOB cells
    EXPECT_EQ(cell_of({-0.5, 1.0}), (Cell{-1, 1})); // negative coordinate
    EXPECT_EQ(cell_of({1.0, 1.0}), (Cell{1, 1}));   // cell boundary
    EXPECT_EQ(cell_of({0.999, 1.0}), (Cell{0, 1})); // cell boundary
}

TEST(Geometry, CenterOfReturnsCorrectCell)
{
    EXPECT_EQ(cell_of(center_of({1, 1})), (Cell{1, 1}));
    EXPECT_EQ(cell_of(center_of({-1, -1})), (Cell{-1, -1}));
}

TEST(Geometry, CorrectDistance)
{
    EXPECT_DOUBLE_EQ(distance({0.0, 0.0}, {3.0, 4.0}), 5.0);
}