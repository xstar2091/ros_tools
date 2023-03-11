#pragma once

#include <unordered_map>

template <class ValueType>
class GraphNode
{
public:
    using value_type = ValueType;

public:
    GraphNode()
        : value_()
    {}
    GraphNode(value_type&& val)
        : value_(std::move(val))
    {}
    GraphNode(const value_type& val)
        : value_(val)
    {}
    GraphNode(GraphNode&&) = delete;
    GraphNode(const GraphNode&) = delete;
    GraphNode& operator=(GraphNode&&) = delete;
    GraphNode& operator=(const GraphNode&) = delete;

public:
    value_type& value() { return value_; }
    const value_type& value() const { return value_; }
    std::unordered_map<value_type, GraphNode<value_type>*>& prev_nodes() { return prev_nodes_; }
    std::unordered_map<value_type, GraphNode<value_type>*>& next_nodes() { return next_nodes_; }

public:
    /**
     * @brief 添加一条本节点指向next_node节点的边
     * @return true  创建一条本节点指向next_node节点的边
     * @return false 本节点指向next_node节点的边已经存在
     */
    bool addEdge(GraphNode* next_node)
    {
        if (isEdgeExist(next_node))
        {
            return false;
        }
        next_nodes_.insert(std::make_pair(next_node->value_, next_node));
        next_node->prev_nodes_.insert(std::make_pair(this->value_, this));
        return true;
    }

    /**
     * @brief 判断本节点指向next_node节点的边是否存在
     * @return true  边存在
     * @return false 边不存在
     */
    bool isEdgeExist(GraphNode* next_node)
    {
        auto next_node_iterator = next_nodes_.find(next_node->value_);
        auto prev_node_iterator = next_node->prev_nodes_.find(this->value_);
        return next_node_iterator != next_nodes_.end() && prev_node_iterator != next_node->prev_nodes_.end();
    }

private:
    value_type value_;
    std::unordered_map<value_type, GraphNode<value_type>*> prev_nodes_;  // 前驱节点集合
    std::unordered_map<value_type, GraphNode<value_type>*> next_nodes_;  // 后继节点集合
};

template <class ValueType>
class Graph
{
public:
    using value_type = ValueType;

public:
    Graph() = default;
    Graph(Graph&&) = delete;
    Graph(const Graph&) = delete;
    Graph& operator=(Graph&&) = delete;
    Graph& operator=(const Graph&) = delete;
    ~Graph()
    {
        for (auto& pair : nodes_)
        {
            delete pair.second;
        }
        nodes_.clear();
    }

public:
    std::unordered_map<value_type, GraphNode<value_type>*>& nodes() { return nodes_; }

public:
    GraphNode<value_type>* createNode(const value_type& val)
    {
        GraphNode<value_type>* node = nullptr;
        auto it = nodes_.find(val);
        if (it == nodes_.end())
        {
            node = new GraphNode<value_type>(val);
            nodes_.insert(std::make_pair(val, node));
        }
        else
        {
            node = it->second;
        }
        return node;
    }

    void createEdge(const value_type& val1, const value_type& val2)
    {
        GraphNode<value_type>* node = createNode(val1);
        GraphNode<value_type>* next = createNode(val2);
        node->addEdge(next);
    }

    bool isEdgeExist(const value_type& val1, const value_type& val2)
    {
        auto it1 = nodes_.find(val1);
        if (it1 == nodes_.end()) return false;
        auto it2 = nodes_.find(val2);
        if (it2 == nodes_.end()) return false;
        return it1->second->isEdgeExist(it2->second);
    }

    bool isNodeExist(const value_type& val)
    {
        auto it = nodes_.find(val);
        return it != nodes_.end();
    }

private:
    std::unordered_map<value_type, GraphNode<value_type>*> nodes_;
};
