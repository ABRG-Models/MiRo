#!/home/dbuxton/mdk/share/python/miro2/dashboard/.venv/bin/python
# -*- coding: utf-8 -*-
# Import Dash components
import dash
import dash_daq as daq
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
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

arrow = {
	'up'   : '⬆',
	'right': '➡',
	'down' : '⬇',
	'left' : '⬅'
}

css = {
	'arrow': {
		'font-size'  : 'xx-large',
		'font-weight': 'bolder',
		'text-align' : 'center',
		'padding'    : '0px'
	}
}

top_arrows = dbc.Row(
	[
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow'],
				id='top-sigma'
			),
			width={
				'size'  : 1,
				'offset': 3
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow'],
				id='top-spatial'
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 3
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow'],
				id='top-affect'
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		)
	],
	no_gutters=True,
)

upper_group = dbc.Row(
	[
		dbc.Col(
			dbc.Alert(
				'Σ',
				style=css['arrow'],
				color='info'
			),
			width={
				'size'  : 1,
				'offset': 3
			}
		),
		dbc.Col(
			html.Div(
				[
					arrow['left'],
					arrow['down']
				]
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader('Action selection'),
					dbc.CardBody(
						[
							dcc.Graph(
								id='action-graph',
								config={
									'displayModeBar': False
								},
								style={
									'height': '600px',
									'width' : '900px',
									'border': '1px solid'
								}
							)
						]
					)
				]
			),
			width={
				'size'  : 5,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow'],
				# id='top-affect'
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
	],
	no_gutters=True
)

upper_arrows = dbc.Row(
	[
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 3
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		)
	],
	no_gutters=True
)

mid_group = dbc.Row(
	[
		dbc.Col(
			dbc.Card(
				[
					dbc.CardBody(dbc.CardText('[MiRo]'))
				]
			),
			width=1
		),
		dbc.Col(
			html.Div(
				arrow['right'],
				style=css['arrow']
			),
			width=1
		),
		dbc.Col(
			[
				dbc.Alert(
					'Motor reafferent',
					color='info'
				),
				dbc.Alert(
					'Noise filter',
					color='info'
				)
			],
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				[
					arrow['right'],
					arrow['down']
				]
			),
			width=1
		),
		dbc.Col(
			html.Div(
				[
					arrow['right'],
					'L'
				]
			),
			width=1
		),
		dbc.Col(
			html.Div(
				[
					arrow['right'],
					arrow['up']
				]
			),
			width=1
		),
		dbc.Col(
			dbc.Card(
				[
					# TODO: Add priority vision, toggle via button
					dbc.CardHeader('Spatial attention'),
					dbc.CardBody(
						[
							dcc.Graph(
								id='cam-graph',
								config={
									'displayModeBar': False
								},
								style={
									'border': '1px solid'
								}
							)
						]
					)
				]
			),
			width={
				'size'  : 2,
				'offset': 0
			}
		),
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader('Affect'),
					dbc.CardBody(
						[
							dcc.Graph(
								id='affect-graph',
								animate=True,
								config={
									'displayModeBar': False
								},
								style={
									'height': '400px',
									'width' : '400px',
									'border': '1px solid'
								}
							)
						]
					)
				]
			),
			width={
				'size'  : 3,
				'offset': 1
			}
		)
	],
	no_gutters=True
)

lower_arrows = dbc.Row(
	[
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 2
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
	],
	no_gutters=True
)

lower_group = dbc.Row(
	[
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader('Body model (Motor)'),
					dbc.CardBody(dbc.CardText('Lorem ipsum'))
				]
			),
			width={
				'size'  : 3,
				'offset': 3
			}
		),
		dbc.Col(
			html.Div(
				arrow['left'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader('Body model (Sensory)'),
					dbc.CardBody(dbc.CardText('Lorem ipsum'))
				]
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader('Circadian rhythm'),
					dbc.CardBody(
						[
							dcc.Graph(
								id='circadian-graph',
								animate=True,
								config={
									'displayModeBar': False
								},
								style={
									'height': '400px',
									'width' : '400px',
									'border': '1px solid'
								}
							)
						]
					)
				]
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader('Expression'),
					dbc.CardBody(dbc.CardText('Lorem ipsum'))
				]
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
	],
	no_gutters=True
)

bottom_arrows = dbc.Row(
	[
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 5
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 1
			}
		),
		dbc.Col(
			html.Div(
				arrow['up'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
		dbc.Col(
			html.Div(
				arrow['down'],
				style=css['arrow']
			),
			width={
				'size'  : 1,
				'offset': 0
			}
		),
	],
	no_gutters=True
)

tooltips = html.Div(
	[
		dbc.Tooltip(
			'Motor pattern output',
			target='top-sigma'
		),
		dbc.Tooltip(
			'Spatial bias, inhibition of return',
			target='top-spatial'
		),
		dbc.Tooltip(
			'Downstream effects on affective state',
			target='top-affect'
		),
	]
)


# Dash Bootstrap CSS theme
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

# Default Dash CSS theme
# app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div(
	[
		top_arrows,
		upper_group,
		upper_arrows,
		mid_group,
		lower_arrows,
		lower_group,
		bottom_arrows,
		tooltips,
		dcc.Interval(
			id='interval-fast',
			interval=0.25 * 1000,
			n_intervals=0
		),
		dcc.Interval(
			id='interval-slow',
			interval=60 * 1000,
			n_intervals=0
		)
	]
)

# This is only to suppress warnings TEMPORARILY
app.config['suppress_callback_exceptions'] = True


@app.callback(Output('action-graph', 'figure'), [Input('interval-fast', 'n_intervals')])
def update_action(n):

	if (miro_ros_data.selection_priority is not None) and (miro_ros_data.selection_inhibition is not None):
		action_channels = [x + 1 for x in range(0, len(miro_ros_data.selection_priority.data))]
		action_inhibition = np.array(miro_ros_data.selection_inhibition.data)
		action_priority = np.array([-x for x in miro_ros_data.selection_priority.data])
	else:
		action_channels = [0]
		action_inhibition = [0]
		action_priority = [0]


	# women_bins = np.array([-600, -623, -653, -650, -670, -578, -541, -411, -322, -230])
	# men_bins = np.array([600, 623, 653, 650, 670, 578, 541, 360, 312, 170])
	#
	# # y = [x + 1 for x in range(0, action_channels)]

	layout = go.Layout(
		yaxis={
			'fixedrange': True,
			'title'     : 'Action'
		},
		xaxis={
			'fixedrange': True,
		    'range'     : [-1, 1],
			'ticktext'  : [1, 0.5, 0, 0.5, 1],
			'tickvals'  : [-1, -0.5, 0, 0.5, 1],
		    'title'     : 'Salience'
		},
		barmode='overlay',
		bargap=0.1
	)

	data = [
		go.Bar(
			y=action_channels,
			x=action_priority,
			orientation='h',
			name='Priority',
			# # FIXME: Get proper action priority value
			# text=abs(action_priority),
			# hoverinfo='text',
			marker={
				'color': 'mediumseagreen'
			}
		),
		go.Bar(
			y=action_channels,
			x=action_inhibition,
			orientation='h',
			name='Inhibition',
			# hoverinfo='x',
			marker={
				'color': 'silver'
			}
		)
	]

	return {
		'data'  : data,
		'layout': layout
	}


@app.callback(Output('affect-graph', 'figure'), [Input('interval-fast', 'n_intervals')])
def update_affect(n):

	affect_layout = go.Layout(
		xaxis={
			'title'         : 'Valence',
			'range'         : [0, 1],
			'fixedrange'    : True,
			'showgrid'      : False,
			'zeroline'      : False,
			'showticklabels': False,
			'mirror'        : True,
			'linewidth'     : 1
		},
		yaxis={
			'title'         : 'Arousal',
			'range'         : [0, 1],
			'fixedrange'    : True,
			'showgrid'      : False,
			'zeroline'      : False,
			'showticklabels': False,
			'mirror'        : True,
			'linewidth'     : 1
		}
	)

	# Update affect
	affect_in = miro_ros_data.core_affect
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
			'data'  : [
				affect_out['emotion'],
				affect_out['mood'],
				affect_out['sleep']
			],
			'layout': affect_layout
		}

	else:
		null_data = go.Scatter(
			x=np.array(-1),
			y=np.array(-1),
		)

		return {
			'data'  : [null_data],
			'layout': affect_layout
		}


@app.callback(Output('cam-graph', 'figure'), [Input('interval-fast', 'n_intervals')])
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


@app.callback(Output('circadian-graph', 'figure'), [Input('interval-slow', 'n_intervals')])
def update_clock_graph(n):

	if miro_ros_data.core_time.data is not None:
		c_data = miro_ros_data.core_time.data
	else:
		c_data = 0

	c_hrs = range(0, 24)
	# Set clock hand width and length
	h_width = 2
	h_length = 0.9

	# TODO: Make circadian clock display leading zeroes
	# circadian_hrs = ['{:02d}'.format(item) for item in range(0, 24)]

	# TODO: Add sun / moon glyphs or background image to plot

	data = [
		go.Scatterpolar(
			fill='toself',
			fillcolor='steelblue',
			marker={
				'line': {
					'color': 'black',
					'width': 0.5
				}
			},
			mode='lines',
			r=[0, 0.1, h_length, 0.1, 0],
			theta=[
				0,
				c_hrs[c_data - h_width],
				c_hrs[c_data],
				c_hrs[c_data + h_width],
				0
			]
		)
	]

	layout = go.Layout(
		# margin={
		# 	'l': 0,
		# 	'r': 0,
		# 	't': 0,
		# 	'b': 0
		# },
		polar={
			'angularaxis': {
				'categoryarray': range(0, 24),
				'direction'    : 'clockwise',
				'nticks'       : 8,
				'period'       : 15,
				'rotation'     : 270,
				'showgrid'     : False,
				'type'         : 'category'
			},
			'radialaxis' : {
				'range'  : [0, 1],
				'visible': False
			}
		},
		showlegend=False
	)

	return {
		'data'  : data,
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
