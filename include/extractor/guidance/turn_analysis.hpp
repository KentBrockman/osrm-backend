#ifndef OSRM_EXTRACTOR_TURN_ANALYSIS
#define OSRM_EXTRACTOR_TURN_ANALYSIS

#include "extractor/compressed_edge_container.hpp"
#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/intersection_generator.hpp"
#include "extractor/guidance/intersection_normalizer.hpp"
#include "extractor/guidance/motorway_handler.hpp"
#include "extractor/guidance/roundabout_handler.hpp"
#include "extractor/guidance/sliproad_handler.hpp"
#include "extractor/guidance/toolkit.hpp"
#include "extractor/guidance/turn_classification.hpp"
#include "extractor/guidance/turn_handler.hpp"
#include "extractor/query_node.hpp"
#include "extractor/restriction_map.hpp"
#include "extractor/suffix_table.hpp"

#include "util/attributes.hpp"
#include "util/name_table.hpp"
#include "util/node_based_graph.hpp"

#include <cstdint>

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/optional.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

class TurnAnalysis
{

  public:
    TurnAnalysis(const util::NodeBasedDynamicGraph &node_based_graph,
                 const std::vector<QueryNode> &node_info_list,
                 const RestrictionMap &restriction_map,
                 const std::unordered_set<NodeID> &barrier_nodes,
                 const CompressedEdgeContainer &compressed_edge_container,
                 const util::NameTable &name_table,
                 const SuffixTable &street_name_suffix_table,
                 const ProfileProperties &profile_properties);

    /* Full Analysis Process for a single node/edge combination. Use with caution, as the process is
     * relatively expensive */
    OSRM_ATTR_WARN_UNUSED
    Intersection operator()(const NodeID node_prior_to_intersection,
                            const EdgeID entering_via_edge) const;

    /*
     * Returns a normalized intersection without any assigned turn types.
     * This intersection can be used as input for intersection classification, turn lane assignment
     * and similar.
     */
    OSRM_ATTR_WARN_UNUSED
    std::tuple<Intersection, Intersection, std::unordered_map<EdgeID, EdgeID>>
    ComputeIntersectionShapes(const NodeID node_at_center_of_intersection) const;

    /*
     * To be used in the road network, we need to check for valid/restricted turns. These two
     * functions transform a basic intersection / a normalised intersection into the correct views
     * when entering via a given edge.
     */
    OSRM_ATTR_WARN_UNUSED
    Intersection AssignTurnAnglesAndValidTags(const NodeID previous_node,
                                              const EdgeID entering_via_edge,
                                              Intersection intersection) const;
    // version for normalised intersection
    OSRM_ATTR_WARN_UNUSED
    Intersection
    AssignTurnAnglesAndValidTags(const NodeID previous_node,
                                 const EdgeID entering_via_edge,
                                 Intersection normalised_intersection,
                                 const Intersection &intersection,
                                 const std::unordered_map<EdgeID, EdgeID> &merging_map) const;
    /*
     * Post-Processing a generated intersection is useful for any intersection that was simply
     * generated using an intersection generator. In the normal use case, you don't have to call
     * this function.
     * This function is part of the normal process of the operator().
     */
    OSRM_ATTR_WARN_UNUSED
    Intersection
    PostProcess(const NodeID from_node, const EdgeID via_eid, Intersection intersection) const;

    std::vector<TurnOperation>
    transformIntersectionIntoTurns(const Intersection &intersection) const;

    Intersection
    assignTurnTypes(const NodeID from_node, const EdgeID via_eid, Intersection intersection) const;

    const IntersectionGenerator &GetIntersectionGenerator() const;

  private:
    const util::NodeBasedDynamicGraph &node_based_graph;
    const RestrictionMap &restriction_map;
    const std::unordered_set<NodeID> &barrier_nodes;

    const IntersectionGenerator intersection_generator;
    const IntersectionNormalizer intersection_normalizer;
    const RoundaboutHandler roundabout_handler;
    const MotorwayHandler motorway_handler;
    const TurnHandler turn_handler;
    const SliproadHandler sliproad_handler;

    // Utility function, setting basic turn types. Prepares for normal turn handling.
    Intersection
    setTurnTypes(const NodeID from, const EdgeID via_edge, Intersection intersection) const;

    // checks turn restrictions for only allowed turn (if one exists)
    boost::optional<NodeID> GetOnlyAllowedTurnIfExistent(const NodeID coming_from_node,
                                                         const NodeID node_at_intersection) const;
}; // class TurnAnalysis

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_EXTRACTOR_TURN_ANALYSIS
