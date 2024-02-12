#include "Junction.h"

namespace odr
{

JunctionLaneLink::JunctionLaneLink(int from, int to, int next) : from(from), to(to), next(next) {}

JunctionConnection::JunctionConnection(std::string id, std::string incoming_road, std::string connecting_road, ContactPoint contact_point, std::string outgoing_road) :
    id(id),incoming_road(incoming_road), connecting_road(connecting_road), contact_point(contact_point), outgoing_road(outgoing_road)
{
}

JunctionPriority::JunctionPriority(std::string high, std::string low) : high(high), low(low) {}

JunctionController::JunctionController(std::string id, std::string type, std::uint32_t sequence) : id(id), type(type), sequence(sequence) {}

Junction::Junction(std::string name, std::string id) : name(name), id(id) {}

} // namespace odr