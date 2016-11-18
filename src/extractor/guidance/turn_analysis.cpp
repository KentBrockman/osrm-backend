#include "extractor/guidance/turn_analysis.hpp"
#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/road_classification.hpp"

#include "util/coordinate.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/guidance/toolkit.hpp"
#include "util/simple_logger.hpp"

#include <cstddef>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

using osrm::util::guidance::getTurnDirection;

namespace osrm
{
namespace extractor
{
namespace guidance
{

using EdgeData = util::NodeBasedDynamicGraph::EdgeData;

bool requiresAnnouncement(const EdgeData &from, const EdgeData &to)
{
    return !from.CanCombineWith(to);
}

TurnAnalysis::TurnAnalysis(const util::NodeBasedDynamicGraph &node_based_graph,
                           const std::vector<QueryNode> &node_info_list,
                           const RestrictionMap &restriction_map,
                           const std::unordered_set<NodeID> &barrier_nodes,
                           const CompressedEdgeContainer &compressed_edge_container,
                           const util::NameTable &name_table,
                           const SuffixTable &street_name_suffix_table,
                           const ProfileProperties &profile_properties)
    : node_based_graph(node_based_graph), restriction_map(restriction_map),
      barrier_nodes(barrier_nodes), intersection_generator(node_based_graph,
                                                           restriction_map,
                                                           barrier_nodes,
                                                           node_info_list,
                                                           compressed_edge_container),
      intersection_normalizer(node_based_graph,
                              node_info_list,
                              name_table,
                              street_name_suffix_table,
                              intersection_generator),
      roundabout_handler(node_based_graph,
                         node_info_list,
                         compressed_edge_container,
                         name_table,
                         street_name_suffix_table,
                         profile_properties,
                         intersection_generator),
      motorway_handler(node_based_graph,
                       node_info_list,
                       name_table,
                       street_name_suffix_table,
                       intersection_generator),
      turn_handler(node_based_graph,
                   node_info_list,
                   name_table,
                   street_name_suffix_table,
                   intersection_generator),
      sliproad_handler(intersection_generator,
                       node_based_graph,
                       node_info_list,
                       name_table,
                       street_name_suffix_table)
{
}

Intersection TurnAnalysis::operator()(const NodeID node_prior_to_intersection,
                                      const EdgeID entering_via_edge) const
{
    Intersection intersection_shape, normalised_shape;
    std::unordered_map<EdgeID, EdgeID> normalisation_map;
    std::tie(intersection_shape, normalised_shape, normalisation_map) =
        ComputeIntersectionShapes(node_based_graph.GetTarget(entering_via_edge));

    // assign valid flags to normalised_shape
    normalised_shape = AssignTurnAnglesAndValidTags(node_prior_to_intersection,
                                                    entering_via_edge,
                                                    std::move(normalised_shape),
                                                    intersection_shape,
                                                    normalisation_map);

    // assign the turn types to the intersection
    return assignTurnTypes(
        node_prior_to_intersection, entering_via_edge, std::move(normalised_shape));
}

Intersection TurnAnalysis::assignTurnTypes(const NodeID node_prior_to_intersection,
                                           const EdgeID entering_via_edge,
                                           Intersection intersection) const
{
    // Roundabouts are a main priority. If there is a roundabout instruction present, we process the
    // turn as a roundabout
    if (roundabout_handler.canProcess(node_prior_to_intersection, entering_via_edge, intersection))
    {
        intersection = roundabout_handler(
            node_prior_to_intersection, entering_via_edge, std::move(intersection));
    }
    else
    {
        // set initial defaults for normal turns and modifier based on angle
        intersection =
            setTurnTypes(node_prior_to_intersection, entering_via_edge, std::move(intersection));
        if (motorway_handler.canProcess(
                node_prior_to_intersection, entering_via_edge, intersection))
        {
            intersection = motorway_handler(
                node_prior_to_intersection, entering_via_edge, std::move(intersection));
        }
        else
        {
            BOOST_ASSERT(turn_handler.canProcess(
                node_prior_to_intersection, entering_via_edge, intersection));
            intersection = turn_handler(
                node_prior_to_intersection, entering_via_edge, std::move(intersection));
        }
    }
    // Handle sliproads
    if (sliproad_handler.canProcess(node_prior_to_intersection, entering_via_edge, intersection))
        intersection = sliproad_handler(
            node_prior_to_intersection, entering_via_edge, std::move(intersection));

    // Turn On Ramps Into Off Ramps, if we come from a motorway-like road
    if (node_based_graph.GetEdgeData(entering_via_edge).road_classification.IsMotorwayClass())
    {
        std::for_each(intersection.begin(), intersection.end(), [](ConnectedRoad &road) {
            if (road.instruction.type == TurnType::OnRamp)
                road.instruction.type = TurnType::OffRamp;
        });
    }
    return intersection;
}

std::vector<TurnOperation>
TurnAnalysis::transformIntersectionIntoTurns(const Intersection &intersection) const
{
    std::vector<TurnOperation> turns;
    for (auto road : intersection)
        if (road.entry_allowed)
            turns.emplace_back(road);

    return turns;
}

std::tuple<Intersection, Intersection, std::unordered_map<EdgeID, EdgeID>>
TurnAnalysis::ComputeIntersectionShapes(const NodeID node_at_center_of_intersection) const
{
    const auto intersection_shape =
        intersection_generator.ComputeIntersectionShape(node_at_center_of_intersection);

    std::unordered_map<EdgeID, EdgeID> merging_map;
    const auto normalized_shape_and_map =
        intersection_normalizer(node_at_center_of_intersection, intersection_shape);

    return std::tie(
        intersection_shape, normalized_shape_and_map.first, normalized_shape_and_map.second);
}
Intersection TurnAnalysis::PostProcess(const NodeID from_node,
                                       const EdgeID entering_via_edge,
                                       Intersection intersection) const
{
    const auto node_at_intersection = node_based_graph.GetTarget(entering_via_edge);
    return assignTurnTypes(
        from_node,
        entering_via_edge,
        intersection_normalizer(node_at_intersection, std::move(intersection)).first);
}

// Sets basic turn types as fallback for otherwise unhandled turns
Intersection TurnAnalysis::setTurnTypes(const NodeID node_prior_to_intersection,
                                        const EdgeID,
                                        Intersection intersection) const
{
    for (auto &road : intersection)
    {
        if (!road.entry_allowed)
            continue;

        const EdgeID onto_edge = road.eid;
        const NodeID to_nid = node_based_graph.GetTarget(onto_edge);

        road.instruction = {TurnType::Turn,
                            (node_prior_to_intersection == to_nid) ? DirectionModifier::UTurn
                                                                   : getTurnDirection(road.angle)};
    }
    return intersection;
}

const IntersectionGenerator &TurnAnalysis::GetIntersectionGenerator() const
{
    return intersection_generator;
}

boost::optional<NodeID>
TurnAnalysis::GetOnlyAllowedTurnIfExistent(const NodeID coming_from_node,
                                           const NodeID node_at_intersection) const
{
    // If only restrictions refer to invalid ways somewhere far away, we rather ignore the
    // restriction than to not route over the intersection at all.
    const auto only_restriction_to_node =
        restriction_map.CheckForEmanatingIsOnlyTurn(coming_from_node, node_at_intersection);
    if (only_restriction_to_node != SPECIAL_NODEID)
    {
        // if the mentioned node does not exist anymore, we don't return it. This checks for broken
        // turn restrictions
        for (const auto onto_edge : node_based_graph.GetAdjacentEdgeRange(node_at_intersection))
            if (only_restriction_to_node == node_based_graph.GetTarget(onto_edge))
                return {only_restriction_to_node};
    }
    // Ignore broken only restrictions.
    return boost::none;
}

Intersection TurnAnalysis::AssignTurnAnglesAndValidTags(const NodeID previous_node,
                                                        const EdgeID entering_via_edge,
                                                        Intersection intersection_shape) const
{
    // requires a copy of the intersection
    std::unordered_map<EdgeID, EdgeID> empty_merging_map;
    return AssignTurnAnglesAndValidTags(previous_node,
                                        entering_via_edge,
                                        intersection_shape, // creates a copy
                                        intersection_shape, // reference to local
                                        empty_merging_map);
}

Intersection TurnAnalysis::AssignTurnAnglesAndValidTags(
    const NodeID previous_node,
    const EdgeID entering_via_edge,
    Intersection normalised_intersection,
    const Intersection &intersection,
    const std::unordered_map<EdgeID, EdgeID> &merging_map) const
{
    BOOST_ASSERT(intersection.valid());
    const auto node_at_intersection = node_based_graph.GetTarget(entering_via_edge);

    // check if there is a single valid turn entering the current intersection
    const auto only_valid_turn = GetOnlyAllowedTurnIfExistent(previous_node, node_at_intersection);

    // barriers change our behaviour regarding u-turns
    const bool is_barrier_node = barrier_nodes.find(node_at_intersection) != barrier_nodes.end();

    const auto connect_to_previous_node = [this, previous_node](const ConnectedRoad &road) {
        return node_based_graph.GetTarget(road.eid) == previous_node;
    };

    // check which of the edges is the u-turn edge
    const auto uturn_edge_itr =
        std::find_if(intersection.begin(), intersection.end(), connect_to_previous_node);
    const auto uturn_edge_at_normalised_intersection_itr = std::find_if(
        normalised_intersection.begin(), normalised_intersection.end(), connect_to_previous_node);

    // there needs to be a connection, otherwise stuff went seriously wrong. Note that this is not
    // necessarily the same id as `entering_via_edge`.
    // In cases where parallel edges are present, we only remember the minimal edge. Both share
    // exactly the same coordinates, so the u-turn is still the best choice here.
    BOOST_ASSERT(uturn_edge_itr != intersection.end());

    const auto is_restricted = [&](const NodeID destination) {
        // check if we have a dedicated destination
        if (only_valid_turn && *only_valid_turn != destination)
            return true;

        // not explicitly forbidden
        return restriction_map.CheckIfTurnIsRestricted(
            previous_node, node_at_intersection, destination);
    };

    const auto is_allowed_turn = [&](const ConnectedRoad &road) {
        const auto &road_data = node_based_graph.GetEdgeData(road.eid);
        const NodeID road_destination_node = node_based_graph.GetTarget(road.eid);

        // reverse edges are never valid turns because the resulting turn would look like this:
        // from_node --via_edge--> node_at_intersection <--onto_edge-- to_node
        // however we need this for capture intersection shape for incoming one-ways
        return !road_data.reversed &&
               // we are not turning over a barrier
               (!is_barrier_node || road_destination_node == previous_node) &&
               // don't allow restricted turns
               !is_restricted(road_destination_node);

    };

    // due to merging of roads, the u-turn might actually not be part of the intersection anymore
    const auto uturn_bearing = [&]() {
        if (merging_map.count(uturn_edge_itr->eid) != 0 &&
            merging_map.find(uturn_edge_itr->eid)->second != uturn_edge_itr->eid)
        {
            const auto merged_into_id = merging_map.find(uturn_edge_itr->eid)->second;
            const auto merged_u_turn =
                std::find_if(normalised_intersection.begin(),
                             normalised_intersection.end(),
                             [&](const ConnectedRoad &road) { return road.eid == merged_into_id; });
            BOOST_ASSERT(merged_u_turn != normalised_intersection.end());
            return util::bearing::reverseBearing(merged_u_turn->bearing);
        }
        else
        {
            BOOST_ASSERT(uturn_edge_at_normalised_intersection_itr !=
                         normalised_intersection.end());
            return util::bearing::reverseBearing(
                uturn_edge_at_normalised_intersection_itr->bearing);
        }
    }();

    for (auto &road : normalised_intersection)
    {
        road.entry_allowed = is_allowed_turn(road);
        road.angle = util::bearing::angleBetweenBearings(uturn_bearing, road.bearing);
    }

    // number of found valid exit roads
    const auto valid_count =
        std::count_if(normalised_intersection.begin(),
                      normalised_intersection.end(),
                      [](const ConnectedRoad &road) { return road.entry_allowed; });

    // in general, we don't wan't to allow u-turns. If we don't look at a barrier, we have to check
    // for dead end streets. These are the only ones that we allow uturns for, next to barriers
    // (which are also kind of a dead end, but we don't have to check these again :))
    if ((uturn_edge_at_normalised_intersection_itr != normalised_intersection.end() &&
         uturn_edge_at_normalised_intersection_itr->entry_allowed && !is_barrier_node &&
         valid_count != 1) ||
        valid_count == 0)
    {
        const auto allow_uturn_at_dead_end = [&]() {
            const auto &uturn_data = node_based_graph.GetEdgeData(uturn_edge_itr->eid);

            // we can't turn back onto oneway streets
            if (uturn_data.reversed)
                return false;

            // don't allow explicitly restricted turns
            if (is_restricted(previous_node))
                return false;

            // we define dead ends as roads that can only be entered via the possible u-turn
            const auto is_bidirectional = [&](const EdgeID entering_via_edge) {
                const auto to_node = node_based_graph.GetTarget(entering_via_edge);
                const auto reverse_edge = node_based_graph.FindEdge(to_node, node_at_intersection);
                BOOST_ASSERT(reverse_edge != SPECIAL_EDGEID);
                return !node_based_graph.GetEdgeData(reverse_edge).reversed;
            };

            const auto bidirectional_edges = [&]() {
                std::uint32_t count = 0;
                for (const auto eid : node_based_graph.GetAdjacentEdgeRange(node_at_intersection))
                    if (is_bidirectional(eid))
                        ++count;
                return count;
            }();

            // Checking for dead-end streets is kind of difficult. There is obvious dead ends
            // (single road connected)
            return bidirectional_edges <= 1;
        }();
        uturn_edge_at_normalised_intersection_itr->entry_allowed = allow_uturn_at_dead_end;
    }
    std::sort(std::begin(normalised_intersection),
              std::end(normalised_intersection),
              std::mem_fn(&ConnectedRoad::compareByAngle));

    BOOST_ASSERT(normalised_intersection[0].angle >= 0. &&
                 normalised_intersection[0].angle < std::numeric_limits<double>::epsilon());

    return normalised_intersection;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
