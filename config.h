/*                                                                                                                                             
 * pigrade - A simple CAN-over-IP gateway
 * Copyright (C) 2016 Matthias Kruk                                                                                                            
 *                                                                                                                                             
 * pigrade is free software; you can redistribute it and/or modify                                                                               
 * it under the terms of the GNU General Public License as published                                                                           
 * by the Free Software Foundation; either version 3, or (at your                                                                              
 * option) any later version.                                                                                                                  
 *                                                                                                                                             
 * pigrade is distributed in the hope that it will be useful, but                                                                                
 * WITHOUT ANY WARRANTY; without even the implied warranty of                                                                                  
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU                                                                           
 * General Public License for more details.                                                                                                    
 *                                                                                                                                             
 * You should have received a copy of the GNU General Public License                                                                           
 * along with pigrade; see the file COPYING.  If not, write to the                                                                               
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,                                                                                
 * Boston, MA 02111-1307, USA.                                                                                                                 
 */


#ifndef __PIGRADE_CONFIG_H
#define __PIGRADE_CONFIG_H

#define CONFIG_MY_NAME        "pigrade pro version ( server client all in one )"

#define CONFIG_INET_PORT      3840
#define CONFIG_INET_BACKLOG   16

#define CONFIG_EPOLL_INITSIZE 18
#define CONFIG_BUFFER_FRAMES  8

#endif /* __PIGRADE_CONFIG_H */
