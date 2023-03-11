#include <gtest/gtest.h>

#include "ros_graph/graph.h"

TEST(Graph, createNode)
{
    Graph<size_t> g;

    EXPECT_TRUE(g.nodes().empty());
    for (size_t i = 1; i <= 5; i++)
    {
        g.createNode(i);
        EXPECT_EQ(g.nodes().size(), i);
        EXPECT_TRUE(g.isNodeExist(i));
    }

    size_t count = g.nodes().size();
    for (size_t i = 1; i <= 5; i++)
    {
        g.createNode(i);
        EXPECT_EQ(g.nodes().size(), count);
        EXPECT_TRUE(g.isNodeExist(i));
    }
}

TEST(Graph, createEdge)
{
    Graph<int> g;

    auto node1 = g.createNode(1);
    auto node2 = g.createNode(2);
    EXPECT_TRUE(node1->prev_nodes().empty());
    EXPECT_TRUE(node1->next_nodes().empty());
    EXPECT_TRUE(node2->prev_nodes().empty());
    EXPECT_TRUE(node2->next_nodes().empty());

    g.createEdge(node1->value(), node2->value());
    EXPECT_TRUE(node1->prev_nodes().empty());
    EXPECT_TRUE(node1->next_nodes().find(node2->value()) != node1->next_nodes().end());
    EXPECT_TRUE(node2->prev_nodes().find(node1->value()) != node2->prev_nodes().end());
    EXPECT_TRUE(node2->next_nodes().empty());

    g.createEdge(node2->value(), node1->value());
    EXPECT_TRUE(node1->prev_nodes().find(node2->value()) != node1->prev_nodes().end());
    EXPECT_TRUE(node1->next_nodes().find(node2->value()) != node1->next_nodes().end());
    EXPECT_TRUE(node2->prev_nodes().find(node1->value()) != node2->prev_nodes().end());
    EXPECT_TRUE(node2->next_nodes().find(node1->value()) != node2->next_nodes().end());
}

TEST(Graph, isEdgeExist)
{
    Graph<int> g;

    auto node1 = g.createNode(1);
    auto node2 = g.createNode(2);
    EXPECT_FALSE(g.isEdgeExist(node1->value(), node1->value()));
    EXPECT_FALSE(g.isEdgeExist(node2->value(), node2->value()));
    EXPECT_FALSE(g.isEdgeExist(node1->value(), node2->value()));
    EXPECT_FALSE(g.isEdgeExist(node2->value(), node1->value()));

    g.createEdge(node1->value(), node2->value());
    EXPECT_FALSE(g.isEdgeExist(node1->value(), node1->value()));
    EXPECT_FALSE(g.isEdgeExist(node2->value(), node2->value()));
    EXPECT_TRUE(g.isEdgeExist(node1->value(), node2->value()));
    EXPECT_FALSE(g.isEdgeExist(node2->value(), node1->value()));

    g.createEdge(node2->value(), node1->value());
    EXPECT_FALSE(g.isEdgeExist(node1->value(), node1->value()));
    EXPECT_FALSE(g.isEdgeExist(node2->value(), node2->value()));
    EXPECT_TRUE(g.isEdgeExist(node1->value(), node2->value()));
    EXPECT_TRUE(g.isEdgeExist(node2->value(), node1->value()));
}
