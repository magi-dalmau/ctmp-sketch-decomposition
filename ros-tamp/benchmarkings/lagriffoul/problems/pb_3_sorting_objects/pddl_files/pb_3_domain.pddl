(define (domain clutter)
(:requirements :strips ::typing)
(:types
  surface
  objs
)
(:predicates (on-surface ?x - objs ?surf - surface)
             (arm-empty)
             (holding ?x - objs)
             (on ?x - objs ?y - objs) )

(:action pickup
  :parameters (?ob - objs ?surf - surface)
  :precondition (and (on-surface ?ob ?surf) (arm-empty))
  :effect (and (holding ?ob) (not (on-surface ?ob ?surf)) 
               (not (arm-empty))))

(:action putdown
  :parameters (?ob - objs ?surf - surface)
  :precondition (holding ?ob)
  :effect (and (arm-empty) (on-surface ?ob ?surf) (not (holding ?ob)))))

                      
