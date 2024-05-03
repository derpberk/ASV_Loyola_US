#variable types

#bool "unsigned 8-bit int"
#byte
#char
#int8
#uint8
#int16
#uint16
#int32
#uint32
#int64 "long"
#uint64 "ulong"
#float32
#float64 "double"
#string "ascii string (4)" "str"
#wstring
#time "secs/nsecs unsigned 32-bit ints" "rospy.Time"
#duration "secs/nsecs signed 32-bit ints"

#you can also define inside another msg like
#geometry_msgs/PoseStamped
#you cannot embed another service inside of a service.

#defining constant variables

#default values
#int32 value 1
#string mode "MANUAL"

#Constants names have to be UPPERCASE
#int32 X=123
#int32 Y=-123
#string FOO="foo"
#string EXAMPLE='bar'

#when defining arrays

#int32[] unbounded_integer_array
#int32[5] five_integers_array
#int32[<=5] up_to_five_integers_array

#string string_of_unbounded_size
#string<=10 up_to_ten_characters_string

#string[<=5] up_to_five_unbounded_strings
#string<=10[] unbounded_array_of_string_up_to_ten_characters each
#string<=10[<=5] up_to_five_strings_up_to_ten_characters_each


#add msg, srv and actions to the makefile
