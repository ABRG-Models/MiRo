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

# Import system and ROS components
import rospy
import numpy as np
import miro_interface as mi
import time


# TODO: Make 'ball spotted!' display

# TODO: Maybe change layout so spatial is wider, affect is smaller, and circadian moves right by one


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

graph_action = dcc.Graph(
	id='action-graph',
	config={
		'displayModeBar': False
	},
	style={
		'height': '176px',
		'width' : '100%',
		# 'border': '1px solid'
	}
)

graph_affect = dcc.Graph(
	id='affect-graph',
	animate=True,
	config={
		'displayModeBar': False
	},
	style={
		'height': '400px',
		'width' : '100%',
		# 'border': '1px solid'
	}
)

graph_aural = dcc.Graph(
	id='aural-graph',
	config={
		'displayModeBar': False
	},
	style={
		# 'height': '400px',
		'width' : '100%',
		# 'border': '1px solid'
	}
)

graph_cameras = dcc.Graph(
	id='camera-graph',
	config={
		'displayModeBar': False
	},
	style={
		# 'height': '200px',
		'width' : '100%',
		# 'border': '1px solid'
	}
)

graph_circadian = dcc.Graph(
	id='circadian-graph',
	animate=True,
	config={
		'displayModeBar': False
	},
	style={
		'height': '100px',
		'width' : '100%',
		# 'border': '1px solid'
	}
)


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
					dbc.CardBody([graph_action]
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
					dbc.CardHeader('Spatial attention'),
					dbc.CardBody(
						[
							graph_aural,
							graph_cameras,
							daq.BooleanSwitch(
								id='cam-toggle',
								label='Visual attention overlay',
								labelPosition='bottom'
							)
						]
					)
				]
			),
			width={
				'size'  : 3,
				'offset': 0
			}
		),
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader('Affect'),
					dbc.CardBody([graph_affect])
				]
			),
			width={
				'size'  : 3,
				'offset': 0
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
					dbc.CardBody([graph_circadian])
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
		# action_channels = [x + 1 for x in range(0, len(miro_ros_data.selection_priority.data))]
		action_inhibition = np.array(miro_ros_data.selection_inhibition.data)
		action_priority = np.array([-x for x in miro_ros_data.selection_priority.data])
	else:
		# action_channels = [0]
		action_inhibition = [0]
		action_priority = [0]

	# TODO: Extract this list automatically from demo code
	action_list = [
		'Mull',
		'Halt',
		'Orient',
		'Approach',
		'Avert',
		'Flee',
		'Retreat'
	]

	layout = go.Layout(
		bargap=0.1,
		barmode='overlay',
		margin={
			'b': 40,
			'l': 60,
			'r': 0,
			't': 0
		},
		xaxis={
			'fixedrange': True,
			'range'     : [-1, 1],
			'ticktext'  : [1, 0.5, 0, 0.5, 1],
			'tickvals'  : [-1, -0.5, 0, 0.5, 1],
			'title'     : 'Salience'
		},
		yaxis={
			'fixedrange': True,
			# 'title'     : 'Action'
		},
	)

	# TODO: Colour each action individually and label actions appropriately
	data = [
		go.Bar(
			y=action_list,
			x=action_priority,
			orientation='h',
			name='Input',
			# # FIXME: Get proper action priority value
			# text=abs(action_priority),
			# hoverinfo='text',
			marker={
				'color': 'mediumseagreen'
			}
		),
		go.Bar(
			y=action_list,
			x=action_inhibition,
			orientation='h',
			name='Output',
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

	layout = go.Layout(
		legend={
			'orientation': 'h',
			'x'          : 0.5,
			'xanchor'    : 'center',
			'y'          : 1.1
		},
		margin={
			'b': 20,
			'l': 20,
			'r': 5,
			't': 0
		},
		xaxis={
			'fixedrange'    : True,
			'linewidth'     : 0.5,
			'mirror'        : True,
			'range'         : [0, 1],
			'showgrid'      : False,
			'showticklabels': False,
			'title'         : 'Valence',
			'zeroline'      : False,
		},
		yaxis={
			'fixedrange'    : True,
			'linewidth'     : 0.5,
			'mirror'        : True,
			'range'         : [0, 1],
			'showgrid'      : False,
			'showticklabels': False,
			'title'         : 'Arousal',
			'zeroline'      : False,
		}
	)

	# Update affect
	affect_data = miro_ros_data.core_affect
	if affect_data is not None:
		data = {
			'emotion': go.Scatter(
				x=np.array(affect_data.emotion.valence),
				y=np.array(affect_data.emotion.arousal),
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
				x=np.array(affect_data.mood.valence),
				y=np.array(affect_data.mood.arousal),
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
				x=np.array(affect_data.sleep.wakefulness),
				y=np.array(affect_data.sleep.pressure),
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
				data['emotion'],
				data['mood'],
				data['sleep']
			],
			'layout': layout
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

@app.callback(Output('aural-graph', 'figure'), [Input('interval-fast', 'n_intervals')])
def update_aural(n):
	priw = miro_ros_data.core_priw

	# Needs to be updated manually if plot width changes; value includes margins
	p_height = 60

	# Set image properties
	if priw is not None:
		priw_image = [{
			'layer'  : 'below',
			'opacity': 1,
			'sizing' : 'stretch',
			'sizex'  : 1,
			'sizey'  : 1,
			'source' : priw,
			'x'      : 0,
			'y'      : 0,
			'xref'   : 'paper',
			'yref'   : 'paper',
			'yanchor': 'bottom'
		}]
	else:
		priw_image = []

	layout = go.Layout(
		height=p_height,
		margin={
			'b': 0,
			'l': 0,
			'r': 0,
			't': 30
		},
		shapes=[
			{
				'line': {
					'color': 'silver',
					'dash' : 'dot',
					'width': 1,
				},
				'type': 'line',
				'x0'  : 0.5,
				'x1'  : 0.5,
				'xref': 'paper',
				'y0'  : 0,
				'y1'  : 1,
				'yref': 'paper'
			}
		],
		images=priw_image,
		title={'text': 'Aural'},
		xaxis={'visible': False},
		yaxis={'visible': False},
	)

	return {'layout': layout}

@app.callback(Output('camera-graph', 'figure'), [Input('interval-fast', 'n_intervals'), Input('cam-toggle', 'on')])
def update_cameras(n, toggle):
	caml = miro_ros_data.sensors_caml
	camr = miro_ros_data.sensors_camr

	pril = miro_ros_data.core_pril
	prir = miro_ros_data.core_prir

	# Needs to be updated manually if plot width changes; value includes margins
	# p_height = 115
	p_height = 185

	# Set camera image properties
	caml_image = {
		'layer'  : 'below',
		'opacity': 1,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,           # Overridden by 'constrain' property but must still be set
		'source' : caml,
		'x'      : 0,
		'y'      : 0,
		'xanchor': 'left',
		'xref'   : 'paper',
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	camr_image = {
		'layer'  : 'below',
		'opacity': 1,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,
		'source' : camr,
		'x'      : 1,
		'y'      : 0,
		'xanchor': 'right',
		'xref'   : 'paper',
		'yanchor': 'bottom',
		'yref'   : 'paper',
	 }

	pril_image = {
		'layer'  : 'above',
		'opacity': 0.5,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,
		'source' : pril,
		'x'      : 0,
		'y'      : 0,
		'xanchor': 'left',
		'xref'   : 'paper',
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	prir_image = {
		'layer'  : 'above',
		'opacity': 0.5,
		'sizing' : 'contain',
		'sizex'  : 0.5,
		'sizey'  : 1,
		'source' : prir,
		'x'      : 1,
		'y'      : 0,
		'xanchor': 'right',
		'xref'   : 'paper',
		'yanchor': 'bottom',
		'yref'   : 'paper',
	}

	# Show vision with attention overlay, vision alone, or nothing
	if (caml is not None) and (camr is not None):
		if toggle:
			cam_images = [
				caml_image,
				camr_image,
				pril_image,
				prir_image
			]
		else:
			cam_images = [
				caml_image,
				camr_image
			]
	else:
		cam_images = []

	layout = go.Layout(
		height=p_height,
		images=cam_images,
		margin={
			'b': 10,
			'l': 0,
			'r': 0,
			't': 60
		},
		shapes=[
			{
				'line': {
					'color': 'black',
					'dash' : 'dot',
					'width': 1,
				},
				'type': 'line',
				'x0'  : 0.5,
				'x1'  : 0.5,
				'xref': 'paper',
				'y0'  : 0,
				'y1'  : 1,
				'yref': 'paper'
			}
		],
		# TODO: Sort out title padding
		title={'text': 'Vision'},
		xaxis={'visible': False},
		yaxis={'visible': False},
	)

	return {'layout': layout}


@app.callback(Output('circadian-graph', 'figure'), [Input('interval-slow', 'n_intervals')])
def update_clock_graph(n):

	if miro_ros_data.core_time.data is not None:
		circ_data = miro_ros_data.core_time.data
	else:
		circ_data = 0

	# TODO: Make circadian clock display leading zeroes
	circ_hrs = range(0, 24)
	# circ_hrs = ['{:02d}'.format(item) for item in range(0, 24)]

	# Set clock hand width and length
	hand_width = 2
	hand_length = 0.9

	# TODO: Add sun / moon glyphs or background image to plot
	# TODO: Find out how to disable zoom

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
			r=[0, 0.1, hand_length, 0.1, 0],
			theta=[
				0,
				circ_hrs[circ_data - hand_width],
				circ_hrs[circ_data],
				circ_hrs[circ_data + hand_width],
				0
			]
		)
	]

	layout = go.Layout(
		margin={
			'b': 20,
			'l': 10,
			'r': 10,
			't': 20
		},
		polar={
			'angularaxis': {
				'categoryarray': circ_hrs,
				'direction'    : 'clockwise',
				'nticks'       : 8,
				'period'       : 15,
				'rotation'     : 270,
				'showgrid'     : False,
				'type'         : 'category'
			},
			'radialaxis' : {
				'range'     : [0, 1],
				'visible'   : False
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
	# miro_dash = {}

	# TODO: Make accessible from elsewhere on LAN

	# Hot reloading seems to cause "IOError: [Errno 11] Resource temporarily unavailable" errors
	app.run_server(debug=False)
