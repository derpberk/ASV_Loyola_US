---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
For messages you define the variables that you want your message to hold

{% highlight ruby %}
#comment
bool state
geometry_msgs/PoseStamped pose
{% endhighlight %}

the comments work as description and are showed in terminal when you use `ros2 interface show <interface>`

for more information about variables check [variables guide](../Example.html)