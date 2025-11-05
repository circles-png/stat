#![feature(mem_copy_fn)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss
)]

use core::panic;
use std::{
    env::args,
    f32::consts::{FRAC_PI_3, FRAC_PI_6},
    fs::File,
    mem::copy,
};

use glam::{Vec2, Vec3, vec2};
use itertools::{Itertools, chain, izip};
use nannou::{
    app,
    color::{BLACK, BLUE, GREY, LIME, ORANGE, RED, WHITE, encoding::Srgb},
    event::{ElementState, Key, MouseButton, MouseScrollDelta},
    prelude::rgb::Rgb,
    winit::event::WindowEvent,
};
use nannou_egui::{
    Egui,
    egui::{self, Area, Color32, ComboBox, DragValue, Frame, Grid, Ui, epaint::util::FloatOrd},
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

const SCALE: f32 = 1000.;
const SUPPORT_SCALE: f32 = 100. / SCALE;
const FORCE_SCALE: f32 = 100. / SCALE;
const MOMENT_SCALE: f32 = 10000. / SCALE;
const INTERNALS_SCALE: f32 = 50000. / SCALE;
const SUPPORT_COLOR: Rgb<Srgb, u8> = LIME;
const LOAD_COLOR: Rgb<Srgb, u8> = RED;
const REACTION_COLOR: Rgb<Srgb, u8> = BLUE;
const MOMENT_COLOR: Rgb<Srgb, u8> = ORANGE;
const MEMBER_COLOR: Rgb<Srgb, u8> = GREY;
const JOINT_COLOR: Rgb<Srgb, u8> = WHITE;

fn main() {
    app(|app| {
        let structure: Structure = args()
            .nth(1)
            .and_then(|path| serde_json::from_reader(File::open(path).unwrap()).ok())
            .unwrap_or_default();
        app.set_exit_on_escape(false);
        let egui = Egui::from_window(
            &app.window(
                app.new_window()
                    .raw_event(|app, model: &mut Model, event| {
                        model.egui.handle_raw_event(event);

                        let previous = model.structure.clone();
                        match event {
                            WindowEvent::MouseWheel { delta, .. } => match delta {
                                MouseScrollDelta::PixelDelta(physical_position) => {
                                    model.offset += Vec2::new(
                                        physical_position.x as f32,
                                        -physical_position.y as f32,
                                    ) / model.zoom;
                                }
                                MouseScrollDelta::LineDelta(_, _) => {}
                            },
                            WindowEvent::TouchpadMagnify { delta, .. } => {
                                model.zoom *= 1. + *delta as f32;
                            }
                            WindowEvent::MouseInput { state, button, .. } => {
                                 if model.egui.ctx().is_pointer_over_area() {
                            return;
                        }
                                let position = (vec2(app.mouse.x, app.mouse.y) / model.zoom
                                    - model.offset)
                                    / SCALE;
                                match (state, button) {
                                    (ElementState::Pressed, MouseButton::Left) => {
                                        model.structure.joints.push(position);
                                    }
                                    (ElementState::Pressed, MouseButton::Right) => {
                                        if let Some((target, _)) = chain!(
                                            model.structure.joints.iter().enumerate().map(
                                                |(index, joint)| {
                                                    (ContextMenuTarget::Joint(index), *joint)
                                                }
                                            ),
                                            model.structure.members.iter().enumerate().map(
                                                |(index, (a, b))| (
                                                    ContextMenuTarget::Member(index),
                                                    model.structure.joints[*a]
                                                        .midpoint(model.structure.joints[*b])
                                                )
                                            )
                                        )
                                        .min_by_key(|(_, joint)| {
                                            position.distance_squared(*joint).ord()
                                        }) {
                                            model.egui.ctx().memory_mut(|memory| {
                                                memory.data.insert_temp(
                                                    "context_menu_target".into(),
                                                    target,
                                                );
                                            });
                                        }
                                    }
                                    (ElementState::Pressed, MouseButton::Middle) => {}
                                    (ElementState::Pressed, MouseButton::Other(_)) => {}
                                    (ElementState::Released, MouseButton::Left) => {}
                                    (ElementState::Released, MouseButton::Right) => {}
                                    (ElementState::Released, MouseButton::Middle) => {}
                                    (ElementState::Released, MouseButton::Other(_)) => {}
                                }
                            }
                            WindowEvent::KeyboardInput { input, .. } => {
                                if input.state == ElementState::Pressed
                                    && input.virtual_keycode == Some(Key::Escape)
                                {
                                    model.egui.ctx().memory_mut(|memory| {
                                        memory.data.remove::<ContextMenuTarget>(
                                            "context_menu_target".into(),
                                        );
                                    });
                                }
                            }
                            _ => {}
                        }
                        if model.structure != previous {
                            model.structure.save();
                            model.external_solution = model.structure.solve_externals();
                            model.internal_solution = model
                                .external_solution
                                .as_ref()
                                .map_err(copy)
                                .map_err(SolveInternalsError::SolveExternalsError)
                                .and_then(|external_solution| {
                                    model.structure.solve_internals(external_solution)
                                });
                        }
                    })
                    .view(|app, model: &Model, frame| {
                        let draw = app
                            .draw()
                            .scale(model.zoom)
                            .translate(model.offset.extend(0.).to_array().into());
                        draw.background().color(BLACK);
                        for (index, (start, end)) in model.structure.members.iter().enumerate() {
                            let start = model.structure.joints[*start] * SCALE;
                            let end = model.structure.joints[*end] * SCALE;
                            draw.line()
                                .start(start.to_array().into())
                                .end(end.to_array().into())
                                .weight(0.01 * SCALE)
                                .color(MEMBER_COLOR);
                            if let Ok(internal_solution) = &model.internal_solution {
                                draw.text(&format!(
                                    "{:.2} ({})",
                                    internal_solution[index].abs(),
                                    if internal_solution[index] > 0. {
                                        "T"
                                    } else {
                                        "C"
                                    }
                                ))
                                .no_line_wrap()
                                .font_size(INTERNALS_SCALE as u32)
                                .xy((start.midpoint(end)).to_array().into());
                            }
                        }
                        let position =
                            (vec2(app.mouse.x, app.mouse.y) / model.zoom - model.offset) / SCALE;
                        let closest_joint =
                            (!model.egui.ctx().is_pointer_over_area())
                                .then_some(())
                                .and_then(|()| {
                                    model.structure.joints.iter().enumerate().min_by_key(
                                        |(_, joint)| joint.distance_squared(position).ord(),
                                    )
                                });
                        for (index, joint) in model.structure.joints.iter().enumerate() {
                            if let Some((closest_index, _)) = closest_joint
                                && index == closest_index
                            {
                                continue;
                            }
                            draw.ellipse()
                                .color(JOINT_COLOR)
                                .radius(0.02 * SCALE)
                                .xy((*joint * SCALE).to_array().into());
                        }
                        for support in &model.structure.supports {
                            let position = model.structure.joints[support.index];
                            match support.kind {
                                SupportKind::Fixed => {}
                                SupportKind::Roller { normal } => {
                                    draw.tri().color(SUPPORT_COLOR).points(
                                        (position * SCALE).to_array(),
                                        ((position
                                            + normal.rotate(
                                                -SUPPORT_SCALE * Vec2::from_angle(-FRAC_PI_6),
                                            ))
                                            * SCALE)
                                            .to_array(),
                                        ((position
                                            + normal.rotate(
                                                -SUPPORT_SCALE * Vec2::from_angle(FRAC_PI_6),
                                            ))
                                            * SCALE)
                                            .to_array(),
                                    );
                                    let radius = SUPPORT_SCALE * SCALE / 3. / 2.;
                                    let center = position * SCALE
                                        - normal * SCALE * SUPPORT_SCALE * 3_f32.sqrt() / 2.
                                        - normal * radius;
                                    for relative_x in [-1., 0., 1.] {
                                        draw.ellipse().color(SUPPORT_COLOR).radius(radius).xy(
                                            (center + normal.perp() * relative_x * 2. * radius)
                                                .to_array()
                                                .into(),
                                        );
                                    }
                                }
                                SupportKind::Pin => {
                                    draw.tri().color(SUPPORT_COLOR).points(
                                        (position * SCALE).to_array(),
                                        ((position + SUPPORT_SCALE * Vec2::from_angle(-FRAC_PI_3))
                                            * SCALE)
                                            .to_array(),
                                        ((position
                                            + SUPPORT_SCALE * Vec2::from_angle(-2. * FRAC_PI_3))
                                            * SCALE)
                                            .to_array(),
                                    );
                                }
                            }
                        }
                        let draw_force = |load: &Load, color| {
                            let point = model.structure.joints[load.index];
                            let vector = load.vector.normalize() * load.vector.length().asinh();
                            let start = ((point - vector * FORCE_SCALE) * SCALE).to_array().into();
                            let end = (point * SCALE).to_array().into();
                            draw.arrow()
                                .color(color)
                                .end(end)
                                .start(start)
                                .caps(nannou::lyon::tessellation::LineCap::Round)
                                .weight(FORCE_SCALE * 150.);
                            draw.text(&format!("{:.2}", load.vector.length()))
                                .no_line_wrap()
                                .font_size((FORCE_SCALE * 1000.) as u32)
                                .xy(start.lerp(end, 0.5));
                        };
                        for load in &model.structure.loads {
                            draw_force(load, LOAD_COLOR);
                        }
                        if let Ok(external_solution) = &model.external_solution {
                            for reaction in &external_solution.reactions {
                                if reaction.vector.length() == 0. {
                                    continue;
                                }
                                draw_force(reaction, REACTION_COLOR);
                            }
                            for (center, moment) in &external_solution.moments {
                                if *moment == 0. {
                                    continue;
                                }
                                let resolution = 32;
                                let start_angle = -FRAC_PI_3;
                                let end_angle = FRAC_PI_3 * 4.;
                                let (start_angle, end_angle) = if *moment > 0. {
                                    (start_angle, end_angle)
                                } else {
                                    (end_angle, start_angle)
                                };
                                let center = center * SCALE;
                                let radius = MOMENT_SCALE * 15.;

                                draw.path()
                                    .stroke()
                                    .color(MOMENT_COLOR)
                                    .stroke_weight(MOMENT_SCALE)
                                    .points(
                                        (0..resolution)
                                            .map(|index| {
                                                start_angle
                                                    + (end_angle - start_angle) * index as f32
                                                        / (resolution - 1) as f32
                                            })
                                            .map(|angle| {
                                                (center + Vec2::from_angle(angle) * radius)
                                                    .to_array()
                                            })
                                            .chain([
                                                (center
                                                    + Vec2::from_angle(
                                                        end_angle
                                                            + if *moment > 0. { -0.3 } else { 0.3 },
                                                    ) * radius
                                                        * 1.2)
                                                    .to_array(),
                                                (center + Vec2::from_angle(end_angle) * radius)
                                                    .to_array(),
                                                (center
                                                    + Vec2::from_angle(
                                                        end_angle
                                                            + if *moment > 0. { -0.3 } else { 0.3 },
                                                    ) * radius
                                                        * 0.8)
                                                    .to_array(),
                                            ]),
                                    );
                                draw.ellipse()
                                    .color(MOMENT_COLOR)
                                    .radius(MOMENT_SCALE)
                                    .xy(center.to_array().into());
                                draw.text(&format!("{moment:.1}"))
                                    .no_line_wrap()
                                    .font_size((FORCE_SCALE * 1000.) as u32)
                                    .xy((center
                                        + Vec2::new(0., MOMENT_SCALE.mul_add(10., radius)))
                                    .to_array()
                                    .into());
                            }
                        }
                        if let Some((index, joint)) = closest_joint {
                            draw.text(&index.to_string())
                                .no_line_wrap()
                                .font_size((FORCE_SCALE * 1000.) as u32)
                                .xy((*joint * SCALE).to_array().into());
                        }
                        draw.to_frame(app, &frame).unwrap();
                        model.egui.draw_to_frame(&frame).unwrap();
                    })
                    .build()
                    .unwrap(),
            )
            .unwrap(),
        );
        let external_solution = structure.solve_externals();
        let internal_solution = external_solution
            .as_ref()
            .map_err(copy)
            .map_err(SolveInternalsError::SolveExternalsError)
            .and_then(|external_solution| structure.solve_internals(external_solution));
        Model {
            external_solution,
            internal_solution,
            zoom: 0.5,
            offset: Vec2::ZERO,
            structure,
            egui,
        }
    })
    .update(|_, model, update| {
        let previous = model.structure.clone();
        model.egui.set_elapsed_time(update.since_start);
        let ctx = model.egui.begin_frame();
        egui::Window::new("Structure parameters").show(&ctx.context(), |ui| {
            ui.vertical(|ui| {
                ui.heading("Joints");
                Grid::new("joints").show(ui, |ui| {
                    for (index, joint) in model.structure.joints.iter_mut().enumerate() {
                        ui.monospace(index.to_string());
                        ui.label("x");
                        ui.add(DragValue::new(&mut joint.x).speed(0.05));
                        ui.label("y");
                        ui.add(DragValue::new(&mut joint.y).speed(0.05));
                        ui.end_row();
                    }
                });
                ui.heading("Members");
                Grid::new("members").show(ui, |ui| {
                    for (index, (a, b)) in model.structure.members.iter_mut().enumerate() {
                        ui.label("from");
                        ComboBox::from_id_source((index, "from")).show_index(
                            ui,
                            a,
                            model.structure.joints.len(),
                            |index| {
                                format!("Joint {} at {:.2}", index, model.structure.joints[index])
                            },
                        );
                        ui.label("to");
                        ComboBox::from_id_source((index, "to")).show_index(
                            ui,
                            b,
                            model.structure.joints.len(),
                            |index| {
                                format!("Joint {} at {:.2}", index, model.structure.joints[index])
                            },
                        );
                        ui.label("reaction");
                        if let Ok(solution) = &model.internal_solution {
                            ui.label(format!(
                                "{:.2} ({})",
                                solution[index].abs(),
                                if solution[index] > 0. { "T" } else { "C" }
                            ));
                        } else {
                            ui.label("-");
                        }
                        ui.end_row();
                    }
                });
                ui.heading("Loads");
                Grid::new("loads").show(ui, |ui| {
                    for (index, load) in model.structure.loads.iter_mut().enumerate() {
                        ui.label("at");
                        ComboBox::from_id_source((index, "load_at")).show_index(
                            ui,
                            &mut load.index,
                            model.structure.joints.len(),
                            |index| {
                                format!("Joint {} at {:.2}", index, model.structure.joints[index])
                            },
                        );
                        ui.label("vector");
                        ui.add(DragValue::new(&mut load.vector.x).speed(1.0));
                        ui.add(DragValue::new(&mut load.vector.y).speed(1.0));
                        if ui.button("Remove").clicked() {
                            model.structure.loads.remove(index);
                            break;
                        }
                        ui.end_row();
                    }
                });
                ui.heading("Supports");
                Grid::new("supports").show(ui, |ui| {
                    for (index, support) in model.structure.supports.iter_mut().enumerate() {
                        ui.label("at");
                        ComboBox::from_id_source((index, "support_at")).show_index(
                            ui,
                            &mut support.index,
                            model.structure.joints.len(),
                            |index| {
                                format!("Joint {} at {:.2}", index, model.structure.joints[index])
                            },
                        );
                        ui.label("kind");
                        ComboBox::from_id_source((index, "kind"))
                            .selected_text(match support.kind {
                                SupportKind::Fixed => "Fixed",
                                SupportKind::Roller { .. } => "Roller",
                                SupportKind::Pin => "Pin",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut support.kind, SupportKind::Fixed, "Fixed");
                                ui.selectable_value(&mut support.kind, SupportKind::Pin, "Pin");
                                ui.selectable_value(
                                    &mut support.kind,
                                    SupportKind::Roller { normal: Vec2::Y },
                                    "Roller",
                                );
                            });

                        if let SupportKind::Roller { normal } = &mut support.kind {
                            ui.label("normal");
                            let mut degrees = normal.to_angle().to_degrees();
                            let mut response =
                                ui.add(DragValue::new(&mut degrees).speed(1.0).suffix("°"));
                            #[allow(clippy::float_cmp, reason = "this checks if `degrees` changed")]
                            if degrees != normal.to_angle().to_degrees() {
                                *normal = Vec2::from_angle(degrees.to_radians());
                                response.changed = true;
                            }
                        } else {
                            ui.label("");
                            ui.label("");
                        }
                        ui.label("reaction");
                        if let Ok(solution) = &model.external_solution {
                            let reaction_force: Vec2 = solution
                                .reactions
                                .iter()
                                .filter(
                                    |Load {
                                         index: joint_index, ..
                                     }| {
                                        *joint_index == support.index
                                    },
                                )
                                .map(|Load { vector, .. }| vector)
                                .sum();
                            let reaction_moment = solution
                                .moments
                                .iter()
                                .filter(|(center, _)| {
                                    *center == model.structure.joints[support.index]
                                })
                                .map(|(_, moment)| moment)
                                .sum1::<f32>();
                            ui.label(format!(
                                "{:.2}, angle {:.2}°",
                                reaction_force.length(),
                                reaction_force.to_angle().to_degrees(),
                            ));
                            if let Some(reaction_moment) = reaction_moment {
                                ui.label(format!("moment {reaction_moment:.2}"));
                            } else {
                                ui.label("");
                            }
                        } else {
                            ui.label("-");
                        }
                        if ui.button("Remove").clicked() {
                            model.structure.supports.remove(index);
                            break;
                        }
                        ui.end_row();
                    }
                });
                if let Err(error) = &model.external_solution {
                    ui.colored_label(Color32::RED, format!("error solving externals: {error}"));
                }
                if let Err(error) = &model.internal_solution {
                    ui.colored_label(Color32::RED, format!("error solving internals: {error}"));
                }
            })
        });
        if let Some(target) =
            ctx.memory(|memory| memory.data.get_temp("context_menu_target".into()))
        {
            let (position, menu) = match target {
                ContextMenuTarget::Joint(index) => {
                    let position = model.structure.joints[index];
                    let structure = &mut model.structure;
                    let ctx = &ctx;
                    (
                        position,
                        Box::new(move |ui: &mut Ui| {
                            ui.heading(format!("Joint {index} at {position:.2}"));
                            let dismiss = || {
                                ctx.memory_mut(|memory| {
                                    memory
                                        .data
                                        .remove::<ContextMenuTarget>("context_menu_target".into());
                                });
                            };
                            if ui.button("Remove joint").clicked() {
                                dismiss();
                                structure.joints.remove(index);
                                structure.members.retain_mut(|(a, b)| {
                                    let retain = *a != index && *b != index;
                                    if retain {
                                        if *a > index {
                                            *a -= 1;
                                        }
                                        if *b > index {
                                            *b -= 1;
                                        }
                                    }
                                    retain
                                });
                                structure.loads.retain_mut(|load| {
                                    let retain = load.index != index;
                                    if retain && load.index > index {
                                        load.index -= 1;
                                    }
                                    retain
                                });
                                structure.supports.retain_mut(|support| {
                                    let retain = support.index != index;
                                    if retain && support.index > index {
                                        support.index -= 1;
                                    }
                                    retain
                                });
                            }
                            if ui.button("Add load here").clicked() {
                                dismiss();
                                structure.loads.push(Load::new(index, vec2(0., -100.)));
                            }
                            if ui.button("Add support here").clicked() {
                                dismiss();
                                structure
                                    .supports
                                    .push(Support::new(index, SupportKind::Pin));
                            }
                            if ui.button("Create member from here").clicked() {
                                dismiss();
                                ctx.memory_mut(|memory| {
                                    memory.data.insert_temp("create_member_start".into(), index);
                                });
                            }
                            if let Some(start) = ctx
                                .memory(|memory| memory.data.get_temp("create_member_start".into()))
                                && start != index
                                && ui
                                    .button(format!("Complete member here (from joint {start})"))
                                    .clicked()
                            {
                                dismiss();
                                ctx.memory_mut(|memory| {
                                    memory.data.remove::<usize>("create_member_start".into());
                                });
                                structure.members.push((start, index));
                            }
                        }) as Box<dyn FnMut(&mut Ui)>,
                    )
                }
                ContextMenuTarget::Member(index) => {
                    let (a, b) = model.structure.members[index];
                    let members = &mut model.structure.members;
                    let ctx = &ctx;
                    (
                        model.structure.joints[a].midpoint(model.structure.joints[b]),
                        Box::new(move |ui: &mut Ui| {
                            let (a, b) = members[index];
                            ui.heading(format!("Member between joints {a} and {b}"));
                            if ui.button("Remove member").clicked() {
                                ctx.memory_mut(|memory| {
                                    memory
                                        .data
                                        .remove::<ContextMenuTarget>("context_menu_target".into());
                                });
                                members.remove(index);
                            }
                        }) as Box<dyn FnMut(&mut Ui)>,
                    )
                }
                ContextMenuTarget::Load(index) => todo!(),
                ContextMenuTarget::Support(index) => todo!(),
            };
            Area::new("context_menu")
                .current_pos(
                    ((position * SCALE + model.offset) * model.zoom * vec2(1., -1.) + {
                        let center = ctx.screen_rect().center();
                        vec2(center.x, center.y)
                    })
                    .to_array(),
                )
                .show(&ctx, |ui| {
                    Frame::group(ui.style())
                        .fill(ui.style().visuals.window_fill())
                        .show(ui, menu)
                        .response
                });
        }
        ctx.end();
        if model.structure != previous {
            model.structure.save();
            model.external_solution = model.structure.solve_externals();
            model.internal_solution = model
                .external_solution
                .as_ref()
                .map_err(copy)
                .map_err(SolveInternalsError::SolveExternalsError)
                .and_then(|external_solution| model.structure.solve_internals(external_solution));
        }
    })
    .run();
}

#[derive(Debug, Clone, Copy, Error)]
enum SolveInternalsError {
    #[error("failed to solve externals: {0}")]
    SolveExternalsError(SolveExternalsError),
    #[error("the internal forces are unsolvable")]
    Unsolvable,
}

struct Model {
    zoom: f32,
    offset: Vec2,
    external_solution: Result<ExternalSolution, SolveExternalsError>,
    internal_solution: Result<Vec<f32>, SolveInternalsError>,
    structure: Structure,
    egui: Egui,
}

#[derive(Clone, Copy)]
enum ContextMenuTarget {
    Joint(usize),
    Member(usize),
    Load(usize),
    Support(usize),
}

#[derive(Debug, Clone)]
struct ExternalSolution {
    reactions: Vec<Load>,
    moments: Vec<(Vec2, f32)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
struct Support {
    index: usize,
    kind: SupportKind,
}

impl Support {
    const fn new(index: usize, kind: SupportKind) -> Self {
        Self { index, kind }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
struct Load {
    index: usize,
    vector: Vec2,
}

impl Load {
    const fn new(index: usize, vector: Vec2) -> Self {
        Self { index, vector }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
enum SupportKind {
    Fixed,
    Roller { normal: Vec2 },
    Pin,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize, Default)]
struct Structure {
    joints: Vec<Vec2>,
    members: Vec<(usize, usize)>,
    loads: Vec<Load>,
    supports: Vec<Support>,
}

impl Structure {
    fn save(&self) {
        let Some(path) = args().nth(1) else {
            return;
        };
        serde_json::to_writer(File::create(path).unwrap(), self).unwrap();
    }
}

#[derive(Debug, Clone, Copy, Error)]
enum SolveExternalsError {
    #[error("the structure is statically indeterminate")]
    StaticallyIndeterminate,
    #[error("the external forces are unsolvable")]
    Unsolvable,
}

impl Structure {
    fn solve_externals(&self) -> Result<ExternalSolution, SolveExternalsError> {
        #[derive(Debug)]
        enum UnknownKind {
            Force { direction: Vec2 },
            Moment,
        }

        struct Unknown {
            support_index: usize,
            kind: UnknownKind,
        }

        let sum_moments: f32 = self
            .loads
            .iter()
            .copied()
            .map(
                |Load {
                     index,
                     vector: force,
                 }| self.joints[index].perp_dot(force),
            )
            .sum();
        let sum_loads: Vec2 = self
            .loads
            .iter()
            .map(|Load { vector: force, .. }| force)
            .sum();
        let mut unknowns = [const { None }; 3];
        {
            let mut writer = unknowns.iter_mut();
            for (support_index, Support { kind, .. }) in self.supports.iter().copied().enumerate() {
                let mut next_unknown = || {
                    writer
                        .next()
                        .ok_or(SolveExternalsError::StaticallyIndeterminate)
                };
                match kind {
                    SupportKind::Fixed => {
                        *next_unknown()? = Some(Unknown {
                            support_index,
                            kind: UnknownKind::Force { direction: Vec2::X },
                        });
                        *next_unknown()? = Some(Unknown {
                            support_index,
                            kind: UnknownKind::Force { direction: Vec2::Y },
                        });
                        *next_unknown()? = Some(Unknown {
                            support_index,
                            kind: UnknownKind::Moment,
                        });
                    }
                    SupportKind::Roller { normal } => {
                        *next_unknown()? = Some(Unknown {
                            support_index,
                            kind: UnknownKind::Force { direction: normal },
                        });
                    }
                    SupportKind::Pin => {
                        *next_unknown()? = Some(Unknown {
                            support_index,
                            kind: UnknownKind::Force { direction: Vec2::X },
                        });
                        *next_unknown()? = Some(Unknown {
                            support_index,
                            kind: UnknownKind::Force { direction: Vec2::Y },
                        });
                    }
                }
            }
        }
        let mut x = [0.; 3];
        let mut y = [0.; 3];
        let mut m = [0.; 3];
        for (unknown, x, y, m) in izip!(&unknowns, &mut x, &mut y, &mut m) {
            let Some(Unknown {
                support_index,
                kind,
            }) = unknown
            else {
                continue;
            };
            match kind {
                UnknownKind::Force { direction } => {
                    *x = direction.x;
                    *y = direction.y;
                    *m = self.joints[self.supports[*support_index].index].perp_dot(*direction);
                }
                UnknownKind::Moment => {
                    *m = 1.;
                }
            }
        }
        let (mut x, mut y, mut m) = ((x, -sum_loads.x), (y, -sum_loads.y), (m, -sum_moments));
        {
            /// Zero out the `index`th component of `subject` using `from`.
            fn zero(subject: &mut ([f32; 3], f32), from: &([f32; 3], f32), index: usize) {
                let scalar = -subject.0[index] / from.0[index];
                for (s, f) in subject.0.iter_mut().zip(&from.0) {
                    *s += scalar * f;
                }
                subject.1 += scalar * from.1;
            }
            /// Scale the `index`th component of `subject` to 1.
            fn one(subject: &mut ([f32; 3], f32), index: usize) {
                let scalar = 1. / subject.0[index];
                for s in &mut subject.0 {
                    *s *= scalar;
                }
                subject.1 *= scalar;
            }
            zero(&mut y, &x, 0);
            zero(&mut m, &x, 0);
            zero(&mut m, &y, 1);
            zero(&mut x, &y, 1);
            zero(&mut x, &m, 2);
            zero(&mut y, &m, 2);
            one(&mut x, 0);
            one(&mut y, 1);
            one(&mut m, 2);
        }

        if Vec3::from_array(x.0).distance_squared(Vec3::X) > 1e-6
            || Vec3::from_array(y.0).distance_squared(Vec3::Y) > 1e-6
            || Vec3::from_array(m.0).distance_squared(Vec3::Z) > 1e-6
            || !x.1.is_finite()
            || !y.1.is_finite()
            || !m.1.is_finite()
        {
            return Err(SolveExternalsError::Unsolvable);
        }
        let solution = [x.1, y.1, m.1];
        Ok(ExternalSolution {
            reactions: unknowns
                .iter()
                .zip(solution)
                .filter_map(|(unknown, value)| {
                    unknown.as_ref().and_then(|unknown| match unknown.kind {
                        UnknownKind::Force { direction } => {
                            Some((unknown.support_index, value, direction))
                        }
                        UnknownKind::Moment => None,
                    })
                })
                .map(|(support_index, value, direction)| Load {
                    index: self.supports[support_index].index,
                    vector: value * direction,
                })
                .collect(),
            moments: unknowns
                .iter()
                .zip(solution)
                .filter_map(|(unknown, value)| {
                    unknown.as_ref().and_then(|unknown| match unknown.kind {
                        UnknownKind::Moment => Some((
                            self.joints[self.supports[unknown.support_index].index],
                            value,
                        )),
                        UnknownKind::Force { .. } => None,
                    })
                })
                .collect(),
        })
    }

    fn solve_internals(
        &self,
        external_solution: &ExternalSolution,
    ) -> Result<Vec<f32>, SolveInternalsError> {
        let mut solution = self.members.iter().map(|_| None::<f32>).collect_vec();
        loop {
            if let Some(solution) = solution.iter().try_fold(Vec::new(), |mut solution, value| {
                solution.push((*value)?);
                Some(solution)
            }) {
                return Ok(solution);
            }
            let (joint_index, knowns, unknowns) = (0..self.joints.len())
                .find_map(|joint_index| {
                    let mut knowns = Vec::new();
                    let mut unknowns = [(Vec2::ZERO, None); 2];
                    let mut writer = unknowns.iter_mut();
                    for internal in self
                        .members
                        .iter()
                        .enumerate()
                        .filter_map(|(index, (a, b))| {
                            let (at_joint, away_from_joint) =
                                match (*a == joint_index, *b == joint_index) {
                                    (false, false) => return None,
                                    (true, false) => (a, b),
                                    (false, true) => (b, a),
                                    (true, true) => panic!("degenerate member at {}", joint_index),
                                };
                            let tension_direction =
                                self.joints[*away_from_joint] - self.joints[*at_joint];
                            Some(
                                solution[index].map_or(Err((tension_direction, index)), |force| {
                                    Ok(force * tension_direction)
                                }),
                            )
                        })
                    {
                        match internal {
                            Ok(force) => knowns.push(force),
                            Err((direction, index)) => {
                                *writer.next()? = (direction, Some(index));
                            }
                        }
                    }
                    if unknowns.iter().all(|(_, index)| index.is_none()) {
                        return None;
                    }
                    Some((joint_index, knowns, unknowns))
                })
                .ok_or(SolveInternalsError::Unsolvable)?;
            let connected_loads = self
                .loads
                .iter()
                .filter(|Load { index, .. }| *index == joint_index);
            let connected_supports = self
                .supports
                .iter()
                .filter(|Support { index, .. }| *index == joint_index);
            let sum_externals = connected_loads.map(|load| load.vector).sum::<Vec2>()
                + connected_supports
                    .flat_map(|Support { index, .. }| {
                        external_solution
                            .reactions
                            .iter()
                            .filter(
                                move |Load {
                                          index: joint_index, ..
                                      }| { *joint_index == *index },
                            )
                            .map(|Load { vector, .. }| vector)
                    })
                    .sum::<Vec2>()
                + knowns.iter().sum::<Vec2>();
            let coefficients = if unknowns[1].1.is_none() {
                if unknowns[0].0.y == 0. {
                    [-sum_externals.x / unknowns[0].0.x, 0.]
                } else {
                    [-sum_externals.y / unknowns[0].0.y, 0.]
                }
            } else {
                [
                    unknowns[1]
                        .0
                        .y
                        .mul_add(sum_externals.x, -(unknowns[1].0.x * sum_externals.y))
                        / unknowns[0]
                            .0
                            .y
                            .mul_add(unknowns[1].0.x, -(unknowns[0].0.x * unknowns[1].0.y)),
                    unknowns[0]
                        .0
                        .x
                        .mul_add(sum_externals.y, -(unknowns[0].0.y * sum_externals.x))
                        / unknowns[0]
                            .0
                            .y
                            .mul_add(unknowns[1].0.x, -(unknowns[0].0.x * unknowns[1].0.y)),
                ]
            };
            for (coefficient, (_, index)) in coefficients.iter().zip(unknowns) {
                let Some(index) = index else {
                    continue;
                };
                solution[index] = Some(*coefficient);
            }
        }
    }
}
