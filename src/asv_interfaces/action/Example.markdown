---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---
Actions are divided in 3 stages

{% highlight ruby %}
# Request
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
{% endhighlight %}

- the Request is what you give when you call the action
- The Result is the answer that the action sends back when it has finished
- The Feedback are (optional) configurable messages where you can send information about the process happening

Actions have the upside that they can be interrupted or stopped anytime contrary to services

the comments work as description and are showed in terminal when you use `ros2 interface show <interface>`

for more information about variables check [variables guide](../Example.html)
