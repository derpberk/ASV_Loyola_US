---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

Services are divided in 2 parts
{% highlight ruby %}
#request fields
int8 foobar
---
#response fields
uint32 an_integer
{% endhighlight %}

- The first part contains the data send to the service_server in the request field
- The second part contains the response field given by the future when the service call ends

you can add comments to your message showed with `ros2 interface show <interface>`

for more information about variables check [variables guide](../Example.html)