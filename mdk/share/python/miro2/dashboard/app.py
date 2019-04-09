#!/home/dbuxton/miro/Venv/bin/python
# -*- coding: utf-8 -*-
import dash
import dash_core_components as dcc
import dash_html_components as html

import sys
# import rospy
# import miro_msgs
# from miro_msgs.msg import core_state
# import interface_lib as lib
import interface_2 as lib

# # ROS topic root
# topic_root = "/miro"


# def callback_core_affect(self, object):
#
# 	print(self)
# 	print(object)
# 	print('hello')
#
# 	# if not self.active:
# 	# 	return
#
# 	# store object
# 	core_affect = object
# 	print(core_affect)
# 	return core_affect
#
#
# sub_core_affect = rospy.Subscriber(topic_root + "/core/affect", miro.msg.affect_state, callback_core_affect)

# print(core_affect)

miro = lib.MiroClient()

# print(miro.core_affect)


external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div(children=[
    html.H1(children='Hello Dash'),

    html.Div(children='''
        Dash: A web application framework for Python.
    '''),

    dcc.Graph(
        id='example-graph',
        figure={
            'data': [
                {'x': [1, 2, 3], 'y': [4, 1, 2], 'type': 'bar', 'name': 'SF'},
                {'x': [1, 2, 3], 'y': [2, 4, 5], 'type': 'bar', 'name': u'Montréal'},
            ],
            'layout': {
                'title': 'Dash Data Visualization'
            }
        }
    )
])

if __name__ == '__main__':
    app.run_server(debug=True)