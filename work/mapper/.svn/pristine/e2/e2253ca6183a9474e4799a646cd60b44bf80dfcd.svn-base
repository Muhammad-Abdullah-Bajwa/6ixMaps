/* 
 * Copyright 2018 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * OSMID.h
 *
 *  Created on: Jan 5, 2017
 *      Author: jcassidy
 */

#ifndef GOLDEN_SOLUTION_LIBSTREETSDATABASE_SRC_OSMID_H_
#define GOLDEN_SOLUTION_LIBSTREETSDATABASE_SRC_OSMID_H_

#include <cinttypes>
#include <boost/serialization/base_object.hpp>
#include <functional>

// forward declarations of the entity types
class OSMEntity;
class OSMNode;
class OSMWay;
class OSMRelation;




/** OSMID is an opaque typedef around unsigned long long for OSMID to distinguish it from other types, eg sizes & vector indices
 * and prevent unintentional casts.
 */

class OSMID
{
public:
	explicit constexpr OSMID(uint64_t id=-1ULL) : m_osmID(id){}

	OSMID(const OSMID&) = default;
	OSMID& operator=(const OSMID&) = default;

	explicit operator uint64_t() const;

	bool operator==(OSMID rhs) const;
	bool operator!=(OSMID rhs) const;

	bool operator<(OSMID rhs) const;

	bool valid() const;

	static const OSMID Invalid;

private:
	uint64_t		m_osmID=-1ULL;

	friend class boost::serialization::access;
	template<typename Archive>void serialize(Archive& ar,const unsigned)
		{ ar & m_osmID; }
};



inline bool OSMID::operator==(OSMID rhs) 	const { return m_osmID == rhs.m_osmID; 			}
inline bool OSMID::operator!=(OSMID rhs) 	const { return m_osmID != rhs.m_osmID; 			}
inline bool OSMID::operator<(OSMID rhs) 	const { return m_osmID < rhs.m_osmID; 			}

inline OSMID::operator uint64_t() 			const { return m_osmID; 						}

inline bool OSMID::valid() 					const { return *this != OSMID::Invalid; 		}


class TypedOSMID : public OSMID
{
public:
	enum EntityType
	{
	    Unknown = 0,
	    Node,
	    Way,
	    Relation
	};

	TypedOSMID(){}

	TypedOSMID(EntityType type_,OSMID id) :
		OSMID(id),
		m_type(type_)
	{
	}

	EntityType 	type() 	const { return m_type; }



private:
	EntityType m_type=Unknown;
	static const char typeChar[4];

	friend class boost::serialization::access;
	template<class Archive>void serialize(Archive& ar,const unsigned)
		{ ar & boost::serialization::base_object<OSMID>(*this) & m_type; }

	friend std::ostream& operator<<(std::ostream& os,TypedOSMID tid);
};

std::ostream& operator<<(std::ostream& os,OSMID id);
std::ostream& operator<<(std::ostream& os,TypedOSMID tid);




/** std::hash<T> instances for OSMID and TypedOSMID to support unordered_set/_map
 */

namespace std {

template<>struct hash<OSMID>
{
	std::size_t operator()(OSMID id) 		const;
	std::size_t operator()(std::size_t i)	const;
};

inline std::size_t hash<OSMID>::operator()(OSMID id) 		const	{ return std::hash<uint64_t>()(uint64_t(id)); }
inline std::size_t hash<OSMID>::operator()(std::size_t i)	const	{ return i; 						}


template<>struct hash<TypedOSMID>
{
	std::size_t operator()(TypedOSMID tid)	const;
	std::size_t operator()(std::size_t i)	const;
};

inline std::size_t hash<TypedOSMID>::operator()(TypedOSMID tid)	const
	{ return std::hash<uint64_t>()(uint64_t(tid)) | std::hash<typename std::underlying_type<TypedOSMID::EntityType>::type>()(tid.type()); 	}
inline std::size_t hash<TypedOSMID>::operator()(std::size_t i)		const	{ return i;																}

}


#endif /* GOLDEN_SOLUTION_LIBSTREETSDATABASE_SRC_OSMID_H_ */
