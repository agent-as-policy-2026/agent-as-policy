# RoboDojo Evaluation Summary

## Overview (by capability dimension)

Progress † marks incomplete evaluation (126 = 42 tasks × 3 seeds). Cells show `mean ±std` across seeds; scores reflect completed cells only. Policies are sorted by overall Avg SR.

| Policy | Progress | Generalization SR (%) | Generalization Score | Precision SR (%) | Precision Score | Long-Horizon SR (%) | Long-Horizon Score | Memory SR (%) | Memory Score | Open SR (%) | Open Score | Avg SR (%) | Avg Score |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| AGP | 33.3% † (42/126) | 54.67 ±0.00 | 59.79 ±0.00 | 44.00 ±0.00 | 50.90 ±0.00 | 28.75 ±0.00 | 49.48 ±0.00 | 91.00 ±0.00 | 92.22 ±0.00 | 62.25 ±0.00 | 65.75 ±0.00 | 56.13 ±0.00 | 63.63 ±0.00 |

## Generalization: Standard vs Random (25 episodes each)

Each cell shows `mean ±std` across seeds, averaged over the 12 Generalization tasks with completed data for that half. Δ = Random − Standard (mean SR). Sorted by Standard SR.

| Policy | Standard SR (%) | Standard Score | Random SR (%) | Random Score | Δ SR | Δ Score |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| AGP | 58.00 ±0.00 | 62.67 ±0.00 | 51.33 ±0.00 | 56.92 ±0.00 | -6.67 | -5.75 |

### Per-task Standard vs Random

Additional view only: each of the 12 Generalization tasks is listed as the 25-episode standard split and its `_random` sibling. Columns match the per-policy all-task tables. Existing merged 50-episode scores are unchanged.

#### AGP

| Task | Seed 0 SR (%) | Seed 0 Score | Seed 1 SR (%) | Seed 1 Score | Seed 2 SR (%) | Seed 2 Score | Avg SR (%) | Avg Score |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| <nobr>stack_bowls</nobr> | 88 | 89.80 |  |  |  |  |  |  |
| <nobr>stack_bowls_random</nobr> | 88 | 89.20 |  |  |  |  |  |  |
| <nobr>push_T</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| <nobr>push_T_random</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| <nobr>pack_objects_into_box</nobr> | 24 | 44.60 |  |  |  |  |  |  |
| <nobr>pack_objects_into_box_random</nobr> | 24 | 46.00 |  |  |  |  |  |  |
| <nobr>fold_clothes</nobr> | 84 | 87.20 |  |  |  |  |  |  |
| <nobr>fold_clothes_random</nobr> | 64 | 70.40 |  |  |  |  |  |  |
| <nobr>hang_mugs</nobr> | 12 | 27.20 |  |  |  |  |  |  |
| <nobr>hang_mugs_random</nobr> | 0 | 16.80 |  |  |  |  |  |  |
| <nobr>sweep_blocks</nobr> | 84 | 84.00 |  |  |  |  |  |  |
| <nobr>sweep_blocks_random</nobr> | 68 | 68.00 |  |  |  |  |  |  |
| <nobr>pour_liquid_into_cup</nobr> | 12 | 12.00 |  |  |  |  |  |  |
| <nobr>pour_liquid_into_cup_random</nobr> | 16 | 16.00 |  |  |  |  |  |  |
| <nobr>make_toast</nobr> | 12 | 16.00 |  |  |  |  |  |  |
| <nobr>make_toast_random</nobr> | 4 | 8.00 |  |  |  |  |  |  |
| <nobr>arrange_largest_number</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| <nobr>arrange_largest_number_random</nobr> | 84 | 87.00 |  |  |  |  |  |  |
| <nobr>sort_nesting_dolls_by_size</nobr> | 80 | 80.00 |  |  |  |  |  |  |
| <nobr>sort_nesting_dolls_by_size_random</nobr> | 68 | 68.00 |  |  |  |  |  |  |
| <nobr>store_laptop_and_headphones</nobr> | 0 | 11.20 |  |  |  |  |  |  |
| <nobr>store_laptop_and_headphones_random</nobr> | 0 | 13.60 |  |  |  |  |  |  |
| <nobr>stack_blocks</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| <nobr>stack_blocks_random</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| **Avg Standard** | 58.0 | 62.67 |  |  |  |  | 58 ±0 | 62.67 ±0.00 |
| **Avg Random** | 51.3 | 56.92 |  |  |  |  | 51 ±0 | 56.92 ±0.00 |

## AGP

| Task | Seed 0 SR (%) | Seed 0 Score | Seed 1 SR (%) | Seed 1 Score | Seed 2 SR (%) | Seed 2 Score | Avg SR (%) | Avg Score |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| <nobr>align_blocks</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| <nobr>arrange_largest_number</nobr> | 92 | 93.50 |  |  |  |  |  |  |
| <nobr>build_tower</nobr> | 68 | 73.80 |  |  |  |  |  |  |
| <nobr>classify_objects</nobr> | 92 | 94.40 |  |  |  |  |  |  |
| <nobr>classify_objects_by_language</nobr> | 60 | 73.00 |  |  |  |  |  |  |
| <nobr>cover_blocks</nobr> | 96 | 96.00 |  |  |  |  |  |  |
| <nobr>deposit_coin</nobr> | 20 | 33.20 |  |  |  |  |  |  |
| <nobr>fasten_screws</nobr> | 90 | 94.40 |  |  |  |  |  |  |
| <nobr>fill_egg_holder</nobr> | 0 | 12.30 |  |  |  |  |  |  |
| <nobr>fill_pen_holder</nobr> | 2 | 31.20 |  |  |  |  |  |  |
| <nobr>fold_clothes</nobr> | 74 | 78.80 |  |  |  |  |  |  |
| <nobr>general_pickup</nobr> | 96 | 96.00 |  |  |  |  |  |  |
| <nobr>hang_mugs</nobr> | 6 | 22.00 |  |  |  |  |  |  |
| <nobr>imitate_sorting_sequence</nobr> | 80 | 87.30 |  |  |  |  |  |  |
| <nobr>insert_key</nobr> | 0 | 15.00 |  |  |  |  |  |  |
| <nobr>insert_tubes</nobr> | 44 | 60.80 |  |  |  |  |  |  |
| <nobr>make_kong</nobr> | 2 | 2.00 |  |  |  |  |  |  |
| <nobr>make_toast</nobr> | 8 | 12.00 |  |  |  |  |  |  |
| <nobr>match_and_pick_from_conveyor</nobr> | 96 | 96.00 |  |  |  |  |  |  |
| <nobr>organize_table</nobr> | 32 | 74.50 |  |  |  |  |  |  |
| <nobr>pack_objects_into_box</nobr> | 24 | 45.30 |  |  |  |  |  |  |
| <nobr>pick_from_conveyor_by_image</nobr> | 56 | 56.00 |  |  |  |  |  |  |
| <nobr>play_Xylophone</nobr> | 90 | 90.00 |  |  |  |  |  |  |
| <nobr>play_stacking_toy</nobr> | 6 | 45.80 |  |  |  |  |  |  |
| <nobr>play_tic_tac_toe</nobr> | 62 | 77.00 |  |  |  |  |  |  |
| <nobr>plug_in_charger</nobr> | 4 | 4.00 |  |  |  |  |  |  |
| <nobr>pour_balls_into_vase</nobr> | 36 | 36.00 |  |  |  |  |  |  |
| <nobr>pour_by_language</nobr> | 2 | 7.20 |  |  |  |  |  |  |
| <nobr>pour_liquid_into_cup</nobr> | 14 | 14.00 |  |  |  |  |  |  |
| <nobr>press_by_number</nobr> | 84 | 84.00 |  |  |  |  |  |  |
| <nobr>push_T</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| <nobr>put_bottles_into_dustbin</nobr> | 34 | 58.60 |  |  |  |  |  |  |
| <nobr>solve_equation</nobr> | 90 | 90.00 |  |  |  |  |  |  |
| <nobr>sort_nesting_dolls_by_size</nobr> | 74 | 74.00 |  |  |  |  |  |  |
| <nobr>stack_blocks</nobr> | 100 | 100.00 |  |  |  |  |  |  |
| <nobr>stack_blocks_by_language</nobr> | 94 | 94.80 |  |  |  |  |  |  |
| <nobr>stack_bowls</nobr> | 88 | 89.50 |  |  |  |  |  |  |
| <nobr>store_laptop_and_headphones</nobr> | 0 | 12.40 |  |  |  |  |  |  |
| <nobr>store_tools_in_toolbox</nobr> | 0 | 9.00 |  |  |  |  |  |  |
| <nobr>swap_T</nobr> | 94 | 94.00 |  |  |  |  |  |  |
| <nobr>swap_blocks</nobr> | 96 | 96.00 |  |  |  |  |  |  |
| <nobr>sweep_blocks</nobr> | 76 | 76.00 |  |  |  |  |  |  |
| **Avg (per seed)** | 54.3 | 61.90 |  |  |  |  | 54 ±0 | 61.90 ±0.00 |

