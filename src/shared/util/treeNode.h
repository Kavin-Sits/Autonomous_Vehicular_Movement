#include "math/geometry.h"
#include "math/math_util.h"
#include "math/line2d.h"

using geometry::Line2f;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;

class TreeNode {
    public:
    Vector2f point;
    vector<TreeNode*> children;

    TreeNode(){}
    
    TreeNode(Vector2f pt){
        point = pt;
    }

    void addChild(TreeNode* child){
        children.push_back(child);
    }

};