#!/home/dbuxton/mdk/share/python/miro2/dashboard/.venv/bin/python
# -*- coding: utf-8 -*-
# Import Dash components
import dash
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
					dbc.CardBody(dbc.CardText('[Action selection graph]'))
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
									# 'height': '400px',
									# 'width' : '400px',
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
					dbc.CardBody(dbc.CardText('Lorem ipsum'))
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
			id='interval-component',
			interval=0.25 * 1000,
			n_intervals=0
		)
	]
)

# This is only to suppress warnings TEMPORARILY
app.config['suppress_callback_exceptions'] = True

# app.layout = html.Div(children=[
# 	html.H1(children='MiRo dashboard'),
#
# 	html.Div(children='''
#         What's MiRo up to today?
#     '''),
#
# 	# Changing component IDs will cause an error if the dashboard is still open
# 	dcc.Graph(
# 		id='affect-graph',
# 		animate=True,
# 		config={
# 			'displayModeBar': False
# 		},
# 		style={
# 			'height'         : '400px',
# 			'width'          : '400px',
# 			'border'         : '1px solid',
# 		}
# 	),
#
# 	dcc.Graph(
# 		id='cam-graph',
# 		config={
# 			'displayModeBar': False
# 		}
# 	),
#
# 	dcc.Interval(
# 		id='interval-component',
# 		interval=0.25 * 1000,
# 		n_intervals=0
# 	)
# ])



@app.callback(Output('affect-graph', 'figure'), [Input('interval-component', 'n_intervals')])
def update_affect(n):

	affect_layout = go.Layout(
		title={'text': 'Affect'},
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
	app.run_server(debug=True)