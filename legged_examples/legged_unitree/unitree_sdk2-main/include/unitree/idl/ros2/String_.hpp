/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: String_.idl
  Source: String_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_STRING__HPP
#define DDSCXX_STRING__HPP

#include <string>

namespace std_msgs
{
namespace msg
{
namespace dds_
{
class String_
{
private:
 std::string data_;

public:
  String_() = default;

  explicit String_(
    const std::string& data) :
    data_(data) { }

  const std::string& data() const { return this->data_; }
  std::string& data() { return this->data_; }
  void data(const std::string& _val_) { this->data_ = _val_; }
  void data(std::string&& _val_) { this->data_ = _val_; }

  bool operator==(const String_& _other) const
  {
    (void) _other;
    return data_ == _other.data_;
  }

  bool operator!=(const String_& _other) const
  {
    return !(*this == _other);
  }

};

}

}

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::std_msgs::msg::dds_::String_>::getTypeName()
{
  return "std_msgs::msg::dds_::String_";
}

template <> constexpr bool TopicTraits<::std_msgs::msg::dds_::String_>::isSelfContained()
{
  return false;
}

template <> constexpr bool TopicTraits<::std_msgs::msg::dds_::String_>::isKeyless()
{
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template<> constexpr unsigned int TopicTraits<::std_msgs::msg::dds_::String_>::type_map_blob_sz() { return 210; }
template<> constexpr unsigned int TopicTraits<::std_msgs::msg::dds_::String_>::type_info_blob_sz() { return 100; }
template<> inline const uint8_t * TopicTraits<::std_msgs::msg::dds_::String_>::type_map_blob() {
  static const uint8_t blob[] = {
 0x3c,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf1,  0x59,  0x65,  0x6d,  0x08,  0x0a,  0xb9,  0x26, 
 0xa3,  0x33,  0xa6,  0x9d,  0x26,  0x9c,  0xb1,  0x00,  0x24,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x0c,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00,  0x8d,  0x77,  0x7f,  0x38, 
 0x67,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf2,  0xc3,  0xda,  0x80,  0xd6,  0x38,  0xe9,  0x1b, 
 0x9f,  0xad,  0x0a,  0x6b,  0x76,  0x5c,  0xc2,  0x00,  0x4f,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00, 
 0x25,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x1d,  0x00,  0x00,  0x00,  0x73,  0x74,  0x64,  0x5f, 
 0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a, 
 0x3a,  0x53,  0x74,  0x72,  0x69,  0x6e,  0x67,  0x5f,  0x00,  0x00,  0x00,  0x00,  0x1b,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x13,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00, 
 0x05,  0x00,  0x00,  0x00,  0x64,  0x61,  0x74,  0x61,  0x00,  0x00,  0x00,  0x00,  0x22,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0xf2,  0xc3,  0xda,  0x80,  0xd6,  0x38,  0xe9,  0x1b,  0x9f,  0xad,  0x0a,  0x6b, 
 0x76,  0x5c,  0xc2,  0xf1,  0x59,  0x65,  0x6d,  0x08,  0x0a,  0xb9,  0x26,  0xa3,  0x33,  0xa6,  0x9d,  0x26, 
 0x9c,  0xb1, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::std_msgs::msg::dds_::String_>::type_info_blob() {
  static const uint8_t blob[] = {
 0x60,  0x00,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0x59,  0x65,  0x6d,  0x08,  0x0a,  0xb9,  0x26,  0xa3,  0x33,  0xa6,  0x9d, 
 0x26,  0x9c,  0xb1,  0x00,  0x28,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf2,  0xc3,  0xda,  0x80,  0xd6,  0x38,  0xe9,  0x1b,  0x9f,  0xad,  0x0a,  0x6b, 
 0x76,  0x5c,  0xc2,  0x00,  0x53,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPE_DISCOVERY

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::std_msgs::msg::dds_::String_>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::std_msgs::msg::dds_::String_>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::std_msgs::msg::dds_::String_)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::std_msgs::msg::dds_::String_>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::std_msgs::msg::dds_::String_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!write_string(streamer, instance.data(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::std_msgs::msg::dds_::String_& instance, bool as_key) {
  auto &props = get_type_props<::std_msgs::msg::dds_::String_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::std_msgs::msg::dds_::String_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!read_string(streamer, instance.data(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::std_msgs::msg::dds_::String_& instance, bool as_key) {
  auto &props = get_type_props<::std_msgs::msg::dds_::String_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::std_msgs::msg::dds_::String_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!move_string(streamer, instance.data(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::std_msgs::msg::dds_::String_& instance, bool as_key) {
  auto &props = get_type_props<::std_msgs::msg::dds_::String_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::std_msgs::msg::dds_::String_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!max_string(streamer, instance.data(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::std_msgs::msg::dds_::String_& instance, bool as_key) {
  auto &props = get_type_props<::std_msgs::msg::dds_::String_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_STRING__HPP