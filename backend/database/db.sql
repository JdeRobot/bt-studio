--
-- PostgreSQL database dump
--

-- Dumped from database version 17.4 (Debian 17.4-1.pgdg120+2)
-- Dumped by pg_dump version 17.4 (Debian 17.4-1.pgdg120+2)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET transaction_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: projects; Type: TABLE; Schema: public; Owner: user-dev
--

CREATE TABLE public.projects (
    id character varying(100) NOT NULL,
    name character varying(100) NOT NULL,
    creator integer NOT NULL,
    last_modified timestamp with time zone NOT NULL,
    size bigint NOT NULL
);


ALTER TABLE public.projects OWNER TO "user-dev";

--
-- Data for Name: projects; Type: TABLE DATA; Schema: public; Owner: user-dev
--

COPY public.projects (id, name, creator, last_modified, size) FROM stdin;
composition_demo	Composition Demo	1	2023-11-18 12:53:08+00	-1
follow_line_demo	Follow Line Demo	1	2025-10-18 12:53:08+00	-1
global_nav	Global Navigation with BT	1	2025-11-15 12:53:08+00	-1
laser_bump_and_go	Laser Bump and Go	1	2025-11-18 12:53:08+00	-1
library_demo	Library Demo	1	2025-11-18 12:53:08+00	-1
obstacle_avoidance	Obstacle Avoidance	1	2025-11-18 15:53:08+00	-1
receptionist_demo	Receptionist Demo	1	2025-11-18 16:00:08+00	-1
visual_follow_person	Visual Follow Person	1	2025-11-18 16:03:08+00	-1
warehouse_demo	Warehouse Demo	1	2025-11-18 12:53:08+00	-1
\.


--
-- Name: projects projects_pkey; Type: CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.projects
    ADD CONSTRAINT projects_pkey PRIMARY KEY (id);


--
-- PostgreSQL database dump complete
--

