---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
The messages are constituted using the same variable types but are defined in different ways, for more information check
- [message](./msg/Example.html)
- [service](./srv/Example.html)
- [action](./action/Example.html)
{% highlight ruby %}
`remember to add msg, srv and actions to the makefile`
{% endhighlight %}

The variables can be defined in the following ways

{% highlight ruby %}
bool unsigned 8-bit int
byte
char
int8
uint8
int16
uint16
int32
uint32
int64 (long)
uint64 (ulong)
float32
float64 (double)
string
wstring
time (secs/nsecs unsigned 32-bit ints) (rospy.Time)
duration (secs/nsecs signed 32-bit ints)

#you can also define inside another msg like
asv_interfaces/Location Pose
`you cannot embed another service inside of a service.`
`If its not from asv_interfaces You need to add the package as dependency in package.xml and cmakelist.txt`

#adding default values
int32 value 1
string mode "MANUAL"

#defining constant variables
`Constants names have to be UPPERCASE`
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'

#when defining arrays
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_string_up_to_ten_characters each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each

{% endhighlight %}

