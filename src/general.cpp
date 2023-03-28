#include "general.h"

bool operator!=(Transform const &t1, Transform const& t2){
	//printf("%f != %f = %i\t %f != %f = %i\t %f != %f = %i\n",t1.p.x, t2.p.x, t1.p.x != t2.p.x,t1.p.y, t2.p.y, t1.p.y != t2.p.y, t1.q.GetAngle(), t2.q.GetAngle(), t1.q.GetAngle() != t2.q.GetAngle() );
	return t1.p.x != t2.p.x || t1.p.y != t2.p.y || t1.q.GetAngle() != t2.q.GetAngle();
}

// P operator+=(P const & p1, P const & p2){
// 	P result;
// 	result.x = p.
// }

// P& P::operator-(P const & p){
// 	P result;
// 	result.x = this->x-p.x;
// 	result.y = this->y-p.y;
// 	return result;
// }
