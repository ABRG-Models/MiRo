#!/home/dbuxton/mdk/share/python/miro2/dashboard/.venv/bin/python
# Import Dash components
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

import plotly.graph_objs as go
import plotly.plotly as py

import numpy as np

# Import system and ROS components
import rospy
import miro_interface as mi
import time

from PIL import Image
import io

import Queue


import cv2


external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div(children=[
	html.H1(children='MiRo dashboard'),

	html.Div(children='''
        What's MiRo up to today?
    '''),

	# Changing component IDs will cause an error if the dashboard is still open
	dcc.Graph(
		id='affect-graph',
		animate=True,
		config={
			'displayModeBar': False
		},
		style={
			'height': '600px',
			'width' : '600px'
		}
	),

	dcc.Graph(
		id='cam-graph',
		config={
			'displayModeBar': False
		}
	),

	dcc.Interval(
		id='interval-component',
		interval=0.25 * 1000,
		n_intervals=0
	)
])



@app.callback(Output('affect-graph', 'figure'), [Input('interval-component', 'n_intervals')])
def update_affect(n):

	affect_layout = go.Layout(
		title={'text': 'Affect'},
		xaxis={
			'title'     : 'Valence',
			'range'     : [0, 1],
			'fixedrange': True
		},
		yaxis={
			'title'     : 'Arousal',
			'range'     : [0, 1],
			'fixedrange': True
		}
	)

	# Update affect
	#TODO: Try to prevent "IOError: [Errno 11] Resource temporarily unavailable" errors
	# print('Affect queue empty? ' + str(miro_ros_data.core_affect.empty()))

	affect_in = miro_ros_data.core_affect
	# affect_in = miro_ros_data.core_affect.get()
	if affect_in is not None:
		affect_out = {
			'emotion': go.Scatter(
				x=np.array(affect_in.emotion.valence),
				y=np.array(affect_in.emotion.arousal),
				name='Emotion',
				mode='markers',
				opacity=0.7,
				marker={
					'size': 15,
					'line': {
						'width': 0.5,
						'color': 'black'
					}
				}
			),

			'mood'   : go.Scatter(
				x=np.array(affect_in.mood.valence),
				y=np.array(affect_in.mood.arousal),
				name='Mood',
				mode='markers',
				opacity=0.7,
				marker={
					'size': 15,
					'line': {
						'width': 0.5,
						'color': 'black'
					}
				}
			),

			'sleep'  : go.Scatter(
				x=np.array(affect_in.sleep.wakefulness),
				y=np.array(affect_in.sleep.pressure),
				name='Sleep',
				mode='markers',
				opacity=0.7,
				marker={
					'size': 15,
					'line': {
						'width': 0.5,
						'color': 'black'
					}
				}
			)
		}

		return {
			'data'  : [affect_out['emotion'], affect_out['mood'], affect_out['sleep']],
			'layout': affect_layout
		}

	else:
		null_data = go.Scatter(
			x=np.array(0),
			y=np.array(0),
		)

		return {
			'data'  : [null_data],
			'layout': affect_layout
		}


@app.callback(Output('cam-graph', 'figure'), [Input('interval-component', 'n_intervals')])
def update_cameras(n):
	caml = miro_ros_data.sensors_caml
	camr = miro_ros_data.sensors_camr

	# TODO: Get dimensions directly from image data
	img_width = 640     # FIXME: Is actually 320... not sure why this scales properly
	img_height = 176

	if (caml is not None) and (camr is not None):
		layout = go.Layout(
			title={'text': 'MiRo vision'},
			xaxis={
				'visible'   : False,
				'range'     : [0, img_width * 2],
				'fixedrange': True
			},
			yaxis={
				'visible'   : False,
				'range'     : [0, img_height],
				'fixedrange': True
			},
			width=img_width,
			height=img_height,
			margin={'l': 0, 'r': 0, 't': 0, 'b': 0},
			images=[
				{
					'source' : caml,
					'x'      : 0,
					'y'      : img_height,
					'sizex'  : img_width,
					'sizey'  : img_height,
					'xref'   : "x",
					'yref'   : "y",
					# 'sizing' : "stretch",
					'opacity': 1,
					'layer'  : "below"
				},
				{
					'source' : camr,
					'x'      : img_width,
					'y'      : img_height,
					'sizex'  : img_width,
					'sizey'  : img_height,
					'xref'   : "x",
					'yref'   : "y",
	                # 'sizing' : "stretch",
	                'opacity': 1,
	                'layer'  : "below"
				 }
			]
		)
	else:
		layout = go.Layout(
			xaxis={
				'visible'   : False,
				'range'     : [0, img_width * 2],
				'fixedrange': True
			},
			yaxis={
				'visible'   : False,
				'range'     : [0, img_height],
				'fixedrange': True
			},
			width=img_width,
			height=img_height,
			margin={'l': 0, 'r': 0, 't': 0, 'b': 0},
		)

	return {
		'data'  : [{
			'x'     : [0, img_width * 2],
			'y'     : [0, img_height],
			'mode'  : 'markers',
			'marker': {
				'opacity': 0
			}
		}],
		'layout': layout
	}


if __name__ == '__main__':
	# Initialise a new ROS node
	# disable_rostime must be True to work in Pycharm
	rospy.init_node("dash_listener", anonymous=True, disable_rostime=True)

	# Initialise MiRo client
	miro_ros_data = mi.MiroClient()
	miro_dash = {}

	# Hot reloading seems to cause "IOError: [Errno 11] Resource temporarily unavailable" errors
	app.run_server(debug=False)