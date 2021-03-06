﻿using QuadTreePoint;													
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

namespace QuadtreeVisualization
{
	/// <summary>
	/// Interaction logic for MainWindow.xaml
	/// </summary>
	public partial class MainWindow : Window, INotifyPropertyChanged
	{
		enum MouseMode { None, Point, Selection};

		public class TreeData : IQuadTreeData
		{
			public Ellipse Marker;
			public float X { get; set; }
			public float Y { get; set; }

			public TreeData(float in_x, float in_y)
			{
				Marker = new Ellipse()
				{
					Width = 5,
					Height = 5,
					Fill = Brushes.Green
				};

				Marker.SetValue(Canvas.LeftProperty, in_x - 2.5);
				Marker.SetValue(Canvas.TopProperty, in_y - 2.5);
				Marker.SetValue(Panel.ZIndexProperty, 1);
				Marker.IsHitTestVisible = false;

				X = in_x;
				Y = in_y;
			}
		}


		QuadTreeFloatPoint<TreeData> m_quadtree;

		public event PropertyChangedEventHandler PropertyChanged;

		public double SelectionRectangleLeft { get; internal set; } = 100;
		public double SelectionRectangleTop { get; internal set; } = 100;
		public double SelectionRectangleWidth { get; internal set; } = 200;
		public double SelectionRectangleHeight { get; internal set; } = 200;
		public Visibility SelectionRectangleVisibility { get; internal set; } = Visibility.Hidden;
		private MouseMode m_mouse_mode = MouseMode.None;
		private List<TreeData> m_neighbour_data = null;

		public Point m_selection_rectangle_start;
		public bool m_selection_active = false;

		public Random m_rnd = new Random(DateTime.Now.Millisecond);

		public MainWindow()
		{
			InitializeComponent();
			gMain.DataContext = this;

			m_quadtree = new QuadTreeFloatPoint<TreeData>((float)cMainCanvas.Width / 2, (float)cMainCanvas.Width / 2, (float)cMainCanvas.Width / 2, 4);
		}

		private void DrawQuadtree()
		{
			m_quadtree.TraverseNodesAndLeafs(DrawQuadTreeLeaf, DrawQuadTreeNode);
		}

		private void DrawQuadTreeNode(QuadTreeFloatPointRegion in_bounds)
		{
			var rect = new Rectangle()
			{
				Width = in_bounds.HalfSize * 2 + 1,
				Height = in_bounds.HalfSize * 2 + 1,
				Stroke = Brushes.LightGray,
				SnapsToDevicePixels = true
			};

			rect.SetValue(Canvas.LeftProperty, (double)(in_bounds.CenterX - in_bounds.HalfSize));
			rect.SetValue(Canvas.TopProperty, (double)(in_bounds.CenterY - in_bounds.HalfSize));
			rect.SetValue(Panel.ZIndexProperty, 0);
			cMainCanvas.Children.Add(rect);

		}

		private void DrawQuadTreeLeaf(TreeData in_data)
		{
			cMainCanvas.Children.Add((Ellipse)in_data.Marker);
		}

		private void ClearButton_Click(object sender, RoutedEventArgs e)
		{
			m_quadtree.Clear();
			cMainCanvas.Children.Clear();
		}

		private void Canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
		{
			m_selection_rectangle_start = e.GetPosition(cMainCanvas);
			m_mouse_mode = MouseMode.Point;
			Mouse.Capture(cMainCanvas);
		}

		private void Canvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
		{
			var pos = Mouse.GetPosition(cMainCanvas);
			Mouse.Capture(null);

			switch (m_mouse_mode)
			{
				case MouseMode.Point:
					{
						TreeData data = new TreeData((float)pos.X, (float)pos.Y);

						try
						{
							m_quadtree.Insert(data);
						}
						catch(ArgumentException)
						{

						}

						cMainCanvas.Children.Clear();
						DrawQuadtree();

						m_mouse_mode = MouseMode.None;
					}
					break;

				case MouseMode.Selection:
					{
						SelectionRectangleVisibility = Visibility.Hidden;
						NotifyPropertyChanged("SelectionRectangleVisibility");

						ClearRegionMarkers();
						m_selection_active = false;


						float left = (float)Math.Min(m_selection_rectangle_start.X, pos.X);
						float top = (float)Math.Min(m_selection_rectangle_start.Y, pos.Y);
						float width = (float)Math.Abs(m_selection_rectangle_start.X - pos.X);
						float height = (float)Math.Abs(m_selection_rectangle_start.Y - pos.Y);

						foreach (TreeData data in m_quadtree.Query(left, top, width, height))
						{
							m_selection_active = true;
							data.Marker.Fill = Brushes.Red;
						}									 

						m_mouse_mode = MouseMode.None;
					}
					break;
			}

		}

		private void Canvas_MouseMove(object sender, MouseEventArgs e)
		{
			Point current_pos = e.GetPosition(cMainCanvas);

			switch (m_mouse_mode)
			{
				case MouseMode.Point:
					SelectionRectangleLeft = m_selection_rectangle_start.X;
					SelectionRectangleTop = m_selection_rectangle_start.Y;
					SelectionRectangleWidth = 0;
					SelectionRectangleHeight = 0;
					SelectionRectangleVisibility = Visibility.Visible;

					NotifyPropertyChanged("SelectionRectangleLeft");
					NotifyPropertyChanged("SelectionRectangleTop");
					NotifyPropertyChanged("SelectionRectangleWidth");
					NotifyPropertyChanged("SelectionRectangleHeight");
					NotifyPropertyChanged("SelectionRectangleVisibility");
					m_mouse_mode = MouseMode.Selection;
					break;

				case MouseMode.Selection:
					SelectionRectangleWidth = current_pos.X - m_selection_rectangle_start.X;
					SelectionRectangleHeight = current_pos.Y - m_selection_rectangle_start.Y;

					NotifyPropertyChanged("SelectionRectangleWidth");
					NotifyPropertyChanged("SelectionRectangleHeight");
					break;

				case MouseMode.None:
					if (cbNeighbourSearch.IsChecked == true && !m_selection_active)
					{
						// restore data point color
						ClearNeighbourMarkers();

						m_neighbour_data = m_quadtree.QueryNeighbours((float)current_pos.X, (float)current_pos.Y, 3);

						foreach (TreeData data in m_neighbour_data)
						{
							data.Marker.Fill = Brushes.Cyan;
						}
					}
					break;

			}
		}

		private void ClearNeighbourMarkers()
		{
			if (m_neighbour_data != null)
			{
				foreach (TreeData data in m_neighbour_data)
				{
					data.Marker.Fill = Brushes.Green;
				}
			}
		}

		private void NotifyPropertyChanged(string propertyName)
		{
			if (PropertyChanged != null)
				PropertyChanged.Invoke(this, new PropertyChangedEventArgs(propertyName));
		}

		private void CheckBox_Click(object sender, RoutedEventArgs e)
		{
			if(((CheckBox)sender).IsChecked==false)
			{
				ClearNeighbourMarkers();
			}
		}

		private void ClearRegionMarkers()
		{
			foreach (TreeData data in m_quadtree)
			{
				data.Marker.Fill = Brushes.Green;
			}
		}

		private void DeselectButton_Click(object sender, RoutedEventArgs e)
		{
			ClearRegionMarkers();
			m_selection_active = false;
		}

		private void AddRandomButton_Click(object sender, RoutedEventArgs e)
		{												 
			for (int i = 0; i < 100; i++)
			{
				TreeData data = new TreeData((float)(m_rnd.NextDouble()*cMainCanvas.Width), (float)(m_rnd.NextDouble()*cMainCanvas.Height));

				m_quadtree.Insert(data);
			}

			cMainCanvas.Children.Clear();
			DrawQuadtree();

		}

		private void ExitButton_Click(object sender, RoutedEventArgs e)
		{
			Close();
		}

	}
}
